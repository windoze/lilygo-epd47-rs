use esp_hal::{
    Blocking,
    dma::DmaTxBuf,
    dma_tx_buffer,
    gpio::AnyPin,
    gpio::{Level, Output, OutputConfig},
    lcd_cam::{
        LcdCam,
        lcd::i8080::{self, Command},
    },
    peripherals,
    time::Rate,
};

use crate::rmt;

const LINE_BYTES: usize = 240;
const PAPER_S3_LINE_PADDING: usize = 8;
const PAPER_S3_LINE_BYTES: usize = LINE_BYTES + PAPER_S3_LINE_PADDING;
const DMA_BUFFER_SIZE: usize = PAPER_S3_LINE_BYTES;

#[derive(Debug)]
struct ConfigRegister {
    latch_enable: bool,
    power_disable: bool,
    pos_power_enable: bool,
    neg_power_enable: bool,
    stv: bool,
    power_enable: bool, /* scan_direction, see https://github.com/vroland/epdiy/blob/main/src/board/epd_board_lilygo_t5_47.c#L199 */
    mode: bool,
    output_enable: bool,
}

impl Default for ConfigRegister {
    fn default() -> Self {
        Self {
            latch_enable: false,
            power_disable: true,
            pos_power_enable: false,
            neg_power_enable: false,
            stv: true,
            power_enable: false,
            mode: false,
            output_enable: false,
        }
    }
}

#[derive(Debug)]
struct ShiftRegisterWriter<'d> {
    pin_data: Output<'d>,
    pin_clk: Output<'d>,
    pin_str: Output<'d>,
}

impl<'d> ShiftRegisterWriter<'d> {
    fn new(
        data: impl esp_hal::gpio::OutputPin + 'd,
        clk: impl esp_hal::gpio::OutputPin + 'd,
        str: impl esp_hal::gpio::OutputPin + 'd,
    ) -> Self {
        let output_cfg = OutputConfig::default();
        Self {
            pin_data: Output::new(data, Level::High, output_cfg),
            pin_clk: Output::new(clk, Level::High, output_cfg),
            pin_str: Output::new(str, Level::Low, output_cfg),
        }
    }

    fn write(&mut self, config: &ConfigRegister) {
        self.pin_str.set_low();
        self.write_bool(config.output_enable);
        self.write_bool(config.mode);
        self.write_bool(config.power_enable);
        self.write_bool(config.stv);
        self.write_bool(config.neg_power_enable);
        self.write_bool(config.pos_power_enable);
        self.write_bool(config.power_disable);
        self.write_bool(config.latch_enable);
        self.pin_str.set_high();
    }

    #[inline(always)]
    fn write_bool(&mut self, v: bool) {
        self.pin_clk.set_low();
        self.pin_data.set_level(match v {
            true => Level::High,
            false => Level::Low,
        });
        self.pin_clk.set_high();
    }
}

#[derive(Debug)]
struct M5PaperS3Control<'d> {
    pin_pwr: Output<'d>,
    pin_spv: Output<'d>,
    pin_oe: Output<'d>,
    pin_le: Output<'d>,
    pin_ckv: Output<'d>,
}

impl<'d> M5PaperS3Control<'d> {
    fn new(
        pwr: AnyPin<'d>,
        spv: AnyPin<'d>,
        oe: AnyPin<'d>,
        le: AnyPin<'d>,
        ckv: AnyPin<'d>,
    ) -> Self {
        let output_cfg = OutputConfig::default();

        // Match the M5GFX `Bus_EPD` expectations:
        // - PWR: LOW = off, HIGH = on
        // - OE : LOW = off, HIGH = on
        // - SPV: LOW by default; pulsed in beginTransaction
        // - LE : HIGH between scanlines, LOW during transfer
        // - CKV: toggled around scanline transfer
        Self {
            pin_pwr: Output::new(pwr, Level::Low, output_cfg),
            pin_spv: Output::new(spv, Level::Low, output_cfg),
            pin_oe: Output::new(oe, Level::Low, output_cfg),
            pin_le: Output::new(le, Level::High, output_cfg),
            pin_ckv: Output::new(ckv, Level::High, output_cfg),
        }
    }

    fn power_control(&mut self, on: bool) {
        if on {
            self.pin_oe.set_high();
            busy_delay(100 * 240);
            self.pin_pwr.set_high();
            busy_delay(100 * 240);
            self.pin_spv.set_high();
            busy_delay(1_000 * 240);
        } else {
            busy_delay(1_000 * 240);
            self.pin_pwr.set_low();
            busy_delay(10 * 240);
            self.pin_oe.set_low();
            busy_delay(100 * 240);
            self.pin_spv.set_low();
        }
    }

    fn begin_transaction(&mut self) {
        // Mirrors M5GFX `Bus_EPD::beginTransaction`.
        self.pin_le.set_low();

        self.pin_spv.set_low();
        busy_delay(240);

        self.pin_ckv.set_low();
        busy_delay(3 * 240);

        self.pin_ckv.set_high();
        busy_delay(240);

        self.pin_spv.set_high();

        for _ in 0..3 {
            busy_delay(3 * 240);
            self.pin_ckv.set_low();
            busy_delay(3 * 240);
            self.pin_ckv.set_high();
        }
    }

    fn end_transaction(&mut self) {
        // Mirrors M5GFX `Bus_EPD::endTransaction`.
        self.pin_le.set_low();
        self.pin_ckv.set_high();
    }

    fn scanline_begin(&mut self) -> u32 {
        // Mirrors M5GFX `Bus_EPD::writeScanLine` prelude.
        self.pin_le.set_low();
        self.pin_ckv.set_high();
        cycles()
    }

    fn scanline_end(&mut self, output_time: u16, start_cycles: u32) {
        // Ensure we keep CKV asserted for at least `output_time` ticks.
        // In the original LilyGo driver, `output_time` is expressed in RMT
        // ticks with `clk_divider=8` at 80MHz => 10MHz => 0.1µs.
        //
        // At 240MHz CPU clock, 0.1µs ~= 24 cycles.
        let desired_cycles = output_time as u32 * 24;
        let elapsed = cycles().wrapping_sub(start_cycles);
        if elapsed < desired_cycles {
            busy_delay(desired_cycles - elapsed);
        }

        // Mirrors M5GFX `notify_line_done`.
        self.pin_ckv.set_low();
        self.pin_le.set_high();
    }

    fn skip_line(&mut self) {
        // Similar to the original LilyGo `skip()` CKV pulse, but using GPIO.
        self.pin_le.set_low();
        self.pin_ckv.set_high();
        busy_delay(45 * 24);
        self.pin_ckv.set_low();
        self.pin_le.set_high();
    }
}

/// Pin configuration for supported boards.
///
/// - [`PinConfig::LilyGoT5V23`] matches the original upstream crate
/// - [`PinConfig::M5PaperS3`] matches M5Stack PaperS3 (ESP32-S3) wiring
#[derive(Debug)]
pub enum PinConfig<'d> {
    LilyGoT5V23(LilyGoT5V23Pins<'d>),
    M5PaperS3(M5PaperS3Pins<'d>),
}

#[derive(Debug)]
pub struct LilyGoT5V23Pins<'d> {
    pub data0: AnyPin<'d>,
    pub data1: AnyPin<'d>,
    pub data2: AnyPin<'d>,
    pub data3: AnyPin<'d>,
    pub data4: AnyPin<'d>,
    pub data5: AnyPin<'d>,
    pub data6: AnyPin<'d>,
    pub data7: AnyPin<'d>,
    pub cfg_data: AnyPin<'d>,
    pub cfg_clk: AnyPin<'d>,
    pub cfg_str: AnyPin<'d>,
    /// LCD_CAM `DC` pin, used as SPH (start pulse horizontal).
    pub sph: AnyPin<'d>,
    /// LCD_CAM `WRX` pin, used as CL (pixel clock).
    pub cl: AnyPin<'d>,
    /// RMT pin, used as CKV (vertical clock).
    pub ckv: AnyPin<'d>,
}

#[derive(Debug)]
pub struct M5PaperS3Pins<'d> {
    pub data0: AnyPin<'d>,
    pub data1: AnyPin<'d>,
    pub data2: AnyPin<'d>,
    pub data3: AnyPin<'d>,
    pub data4: AnyPin<'d>,
    pub data5: AnyPin<'d>,
    pub data6: AnyPin<'d>,
    pub data7: AnyPin<'d>,
    /// SPH (start pulse horizontal) - LCD_CAM `CS` pin.
    pub sph: AnyPin<'d>,
    /// CL (pixel clock) - LCD_CAM `WRX` pin.
    pub cl: AnyPin<'d>,
    /// CKV (vertical clock).
    pub ckv: AnyPin<'d>,
    /// SPV / STV (start pulse vertical).
    pub spv: AnyPin<'d>,
    /// LE (latch enable).
    pub le: AnyPin<'d>,
    /// OE (output enable).
    pub oe: AnyPin<'d>,
    /// EPD power enable.
    pub pwr: AnyPin<'d>,
}

pub(crate) struct ED047TC1<'d> {
    i8080: Option<i8080::I8080<'d, Blocking>>,
    backend: Backend<'d>,
    dma_buf: Option<DmaTxBuf>,
    line_bytes: usize,
}

#[inline(always)]
fn swizzle_papers3_byte(b: u8) -> u8 {
    // M5GFX / Panel_EPD packs pixels in the opposite 2bpp order compared to the
    // original LilyGo driver:
    // - our pipeline produces: [p0|p1|p2|p3] as 2-bit pairs from LSB→MSB
    // - PaperS3 expects:       [p0|p1|p2|p3] as 2-bit pairs from MSB→LSB
    //
    // Reverse the order of the 2-bit pairs inside each byte:
    // bits 1:0 ↔ 7:6, 3:2 ↔ 5:4.
    ((b & 0x03) << 6) | ((b & 0x0C) << 2) | ((b & 0x30) >> 2) | ((b & 0xC0) >> 6)
}

enum Backend<'d> {
    LilyGo {
        cfg_writer: ShiftRegisterWriter<'d>,
        cfg: ConfigRegister,
        rmt: rmt::Rmt<'d>,
    },
    M5PaperS3 {
        ctrl: M5PaperS3Control<'d>,
    },
}

impl<'d> ED047TC1<'d> {
    pub(crate) fn new(
        pins: PinConfig<'d>,
        dma_channel: impl esp_hal::dma::TxChannelFor<peripherals::LCD_CAM<'d>> + 'd,
        lcd_cam: peripherals::LCD_CAM<'d>,
        rmt: peripherals::RMT<'d>,
    ) -> crate::Result<Self> {
        let lcd_cam = LcdCam::new(lcd_cam);

        // All supported boards use an 8-bit i8080 data bus.
        let (i8080, backend, line_bytes) = match pins {
            PinConfig::LilyGoT5V23(pins) => {
                let config = i8080::Config::default()
                    .with_frequency(Rate::from_mhz(10))
                    .with_cd_idle_edge(false)
                    .with_cd_cmd_edge(true)
                    .with_cd_dummy_edge(false)
                    .with_cd_data_edge(false);

                let i8080 = i8080::I8080::new(lcd_cam.lcd, dma_channel, config)
                    .map_err(crate::Error::I8080Config)?
                    .with_dc(pins.sph)
                    .with_wrx(pins.cl)
                    .with_data0(pins.data0)
                    .with_data1(pins.data1)
                    .with_data2(pins.data2)
                    .with_data3(pins.data3)
                    .with_data4(pins.data4)
                    .with_data5(pins.data5)
                    .with_data6(pins.data6)
                    .with_data7(pins.data7);

                let cfg = ConfigRegister::default();
                let mut cfg_writer = ShiftRegisterWriter::new(
                    pins.cfg_data,
                    pins.cfg_clk,
                    pins.cfg_str,
                );
                cfg_writer.write(&cfg);

                let rmt = rmt::Rmt::new(rmt, pins.ckv)?;

                let line_bytes = LINE_BYTES;

                (
                    i8080,
                    Backend::LilyGo {
                        cfg_writer,
                        cfg,
                        rmt,
                    },
                    line_bytes,
                )
            }
            PinConfig::M5PaperS3(pins) => {
                let M5PaperS3Pins {
                    data0,
                    data1,
                    data2,
                    data3,
                    data4,
                    data5,
                    data6,
                    data7,
                    sph,
                    cl,
                    ckv,
                    spv,
                    le,
                    oe,
                    pwr,
                } = pins;

                // Match M5GFX `Bus_EPD`:
                // - SPH is wired to LCD_CS, not LCD_DC.
                // - There is no meaningful DC line on this bus.
                let config = i8080::Config::default().with_frequency(Rate::from_mhz(16));

                let i8080 = i8080::I8080::new(lcd_cam.lcd, dma_channel, config)
                    .map_err(crate::Error::I8080Config)?
                    .with_cs(sph)
                    .with_wrx(cl)
                    .with_data0(data0)
                    .with_data1(data1)
                    .with_data2(data2)
                    .with_data3(data3)
                    .with_data4(data4)
                    .with_data5(data5)
                    .with_data6(data6)
                    .with_data7(data7);

                let ctrl = M5PaperS3Control::new(pwr, spv, oe, le, ckv);

                // RMT is unused on PaperS3 but still required by the public
                // constructor for compatibility with upstream.
                let _ = rmt;

                (i8080, Backend::M5PaperS3 { ctrl }, PAPER_S3_LINE_BYTES)
            }
        };

        let dma_buf =
            Some(dma_tx_buffer!(DMA_BUFFER_SIZE).map_err(crate::Error::DmaBuffer)?);

        let ctrl = Self {
            i8080: Some(i8080),
            backend,
            dma_buf,
            line_bytes,
        };

        Ok(ctrl)
    }

    pub(crate) fn power_on(&mut self) {
        match &mut self.backend {
            Backend::LilyGo { cfg_writer, cfg, .. } => {
                cfg.power_enable = true;
                cfg.power_disable = false;
                cfg_writer.write(cfg);
                busy_delay(100 * 240);
                cfg.neg_power_enable = true;
                cfg_writer.write(cfg);
                busy_delay(500 * 240);
                cfg.pos_power_enable = true;
                cfg_writer.write(cfg);
                busy_delay(100 * 240);
                cfg.stv = true;
                cfg_writer.write(cfg);
            }
            Backend::M5PaperS3 { ctrl } => ctrl.power_control(true),
        }
    }

    pub(crate) fn power_off(&mut self) {
        match &mut self.backend {
            Backend::LilyGo { cfg_writer, cfg, .. } => {
                cfg.power_enable = false;
                cfg.pos_power_enable = false;
                cfg_writer.write(cfg);
                busy_delay(10 * 240);
                cfg.neg_power_enable = false;
                cfg_writer.write(cfg);
                busy_delay(100 * 240);
                cfg.power_disable = true;
                cfg.mode = false;
                cfg.stv = false;
                cfg_writer.write(cfg);
            }
            Backend::M5PaperS3 { ctrl } => ctrl.power_control(false),
        }
    }

    pub(crate) fn frame_start(&mut self) -> crate::Result<()> {
        match &mut self.backend {
            Backend::LilyGo { cfg_writer, cfg, rmt } => {
                cfg.mode = true;
                cfg_writer.write(cfg);

                rmt.pulse(10, 10, true)?;

                cfg.stv = false;
                cfg_writer.write(cfg);

                rmt.pulse(10000, 1000, false)?;
                cfg.stv = true;
                cfg_writer.write(cfg);

                rmt.pulse(10, 10, true)?;
                rmt.pulse(10, 10, true)?;
                rmt.pulse(10, 10, true)?;
                rmt.pulse(10, 10, true)?;

                cfg.output_enable = true;
                cfg_writer.write(cfg);
                rmt.pulse(10, 10, true)?;

                Ok(())
            }
            Backend::M5PaperS3 { ctrl } => {
                ctrl.begin_transaction();
                Ok(())
            }
        }
    }

    pub(crate) fn skip(&mut self) -> crate::Result<()> {
        match &mut self.backend {
            Backend::LilyGo { rmt, .. } => rmt.pulse(45, 5, false)?,
            Backend::M5PaperS3 { ctrl } => ctrl.skip_line(),
        }
        Ok(())
    }

    pub(crate) fn output_row(&mut self, output_time: u16) -> crate::Result<()> {
        match &mut self.backend {
            Backend::LilyGo { cfg_writer, cfg, rmt } => {
                cfg.latch_enable = true;
                cfg_writer.write(cfg);
                cfg.latch_enable = false;
                cfg_writer.write(cfg);

                rmt.pulse(output_time, 50, false)?;

                let i8080 = self.i8080.take().ok_or(crate::Error::Unknown)?;
                let dma_buf = self.dma_buf.take().ok_or(crate::Error::Unknown)?;

                let tx = i8080
                    .send(Command::<u8>::One(0), 0, dma_buf)
                    .map_err(|(err, i8080, buf)| {
                        self.dma_buf = Some(buf);
                        self.i8080 = Some(i8080);
                        crate::Error::Dma(err)
                    })?;

                let (r, i8080, dma_buf) = tx.wait();
                r.map_err(crate::Error::Dma)?;

                self.i8080 = Some(i8080);
                self.dma_buf = Some(dma_buf);
                Ok(())
            }
            Backend::M5PaperS3 { ctrl } => {
                let start_cycles = ctrl.scanline_begin();

                let i8080 = self.i8080.take().ok_or(crate::Error::Unknown)?;
                let dma_buf = self.dma_buf.take().ok_or(crate::Error::Unknown)?;

                let tx = i8080
                    .send(Command::<u8>::None, 0, dma_buf)
                    .map_err(|(err, i8080, buf)| {
                        self.dma_buf = Some(buf);
                        self.i8080 = Some(i8080);
                        crate::Error::Dma(err)
                    })?;

                let (r, i8080, dma_buf) = tx.wait();
                r.map_err(crate::Error::Dma)?;

                self.i8080 = Some(i8080);
                self.dma_buf = Some(dma_buf);

                ctrl.scanline_end(output_time, start_cycles);
                Ok(())
            }
        }
    }

    pub(crate) fn frame_end(&mut self) -> crate::Result<()> {
        match &mut self.backend {
            Backend::LilyGo { cfg_writer, cfg, rmt } => {
                cfg.output_enable = false;
                cfg_writer.write(cfg);
                cfg.mode = true;
                cfg_writer.write(cfg);

                rmt.pulse(10, 10, true)?;
                rmt.pulse(10, 10, true)?;
                rmt.pulse(10, 10, true)?;
                Ok(())
            }
            Backend::M5PaperS3 { ctrl } => {
                ctrl.end_transaction();
                Ok(())
            }
        }
    }

    pub(crate) fn set_buffer(&mut self, data: &[u8]) -> crate::Result<()> {
        if data.len() > LINE_BYTES {
            return Err(crate::Error::OutOfBounds);
        }

        let mut dma_buf = self.dma_buf.take().ok_or(crate::Error::Unknown)?;
        dma_buf.as_mut_slice().fill(0);
        match &self.backend {
            Backend::M5PaperS3 { .. } => {
                for (dst, &src) in dma_buf.as_mut_slice()[..data.len()]
                    .iter_mut()
                    .zip(data.iter())
                {
                    *dst = swizzle_papers3_byte(src);
                }
            }
            Backend::LilyGo { .. } => {
                dma_buf.as_mut_slice()[..data.len()].copy_from_slice(data);
            }
        }
        dma_buf.set_length(self.line_bytes);
        self.dma_buf = Some(dma_buf);
        Ok(())
    }
}

#[inline(always)]
fn busy_delay(wait_cycles: u32) {
    // `get_cycle_count` is a 32-bit counter and wraps; use the platform helper
    // that handles wrapping arithmetic correctly.
    esp_hal::xtensa_lx::timer::delay(wait_cycles);
}

#[inline(always)]
fn cycles() -> u32 {
    esp_hal::xtensa_lx::timer::get_cycle_count()
}
