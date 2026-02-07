use core::cell::UnsafeCell;

use esp_hal::{
    Blocking,
    gpio::Level,
    gpio::interconnect::PeripheralOutput,
    peripherals::RMT,
    rmt::{Channel, PulseCode, SingleShotTxTransaction, Tx, TxChannelConfig, TxChannelCreator},
    time::Rate,
};

pub(crate) struct Rmt<'d> {
    tx_channel: Option<Channel<'d, Blocking, Tx>>,
    in_flight: Option<SingleShotTxTransaction<'d, 'static, PulseCode>>,
}

struct PulseBuffer(UnsafeCell<[PulseCode; 2]>);

// Safety: only accessed from the display driver on a single core in a
// serialized fashion (we wait for the in-flight TX before mutating the buffer).
unsafe impl Sync for PulseBuffer {}

static PULSE_BUFFER: PulseBuffer =
    PulseBuffer(UnsafeCell::new([PulseCode::end_marker(), PulseCode::end_marker()]));

impl<'d> Rmt<'d> {
    pub(crate) fn new(rmt: RMT<'d>, pin: impl PeripheralOutput<'d>) -> Result<Self, crate::Error> {
        let rmt = esp_hal::rmt::Rmt::new(rmt, Rate::from_mhz(80)).map_err(crate::Error::Rmt)?;
        let tx_channel = rmt
            .channel1
            .configure_tx(
                pin,
                TxChannelConfig::default()
                    .with_clk_divider(8)
                    .with_idle_output_level(Level::Low)
                    .with_idle_output(true)
                    .with_carrier_modulation(false)
                    .with_carrier_level(Level::Low),
            )
            .map_err(crate::Error::Rmt)?;

        Ok(Self {
            tx_channel: Some(tx_channel),
            in_flight: None,
        })
    }

    fn wait_in_flight(&mut self) -> Result<(), crate::Error> {
        let Some(in_flight) = self.in_flight.take() else {
            return Ok(());
        };

        match in_flight.wait() {
            Ok(tx_channel) => {
                self.tx_channel = Some(tx_channel);
                Ok(())
            }
            Err((err, tx_channel)) => {
                self.tx_channel = Some(tx_channel);
                Err(crate::Error::Rmt(err))
            }
        }
    }

    pub(crate) fn pulse(&mut self, high: u16, low: u16, wait: bool) -> Result<(), crate::Error> {
        // We can't start another TX while one is still in progress. Waiting
        // here still allows the caller to overlap a non-waiting pulse with
        // other work (e.g. starting an LCD_CAM DMA transfer) before the next
        // call to `pulse()`.
        self.wait_in_flight()?;

        let tx_channel = self.tx_channel.take().ok_or(crate::Error::Unknown)?;
        let pulse_buffer = unsafe { &mut *PULSE_BUFFER.0.get() };
        pulse_buffer[0] = if high > 0 {
                PulseCode::new(Level::High, high, Level::Low, low)
            } else {
                PulseCode::new(Level::High, low, Level::Low, 0)
            };
        pulse_buffer[1] = PulseCode::end_marker();

        let tx = tx_channel
            .transmit(unsafe { &*PULSE_BUFFER.0.get() })
            .map_err(crate::Error::Rmt)?;

        if wait {
            match tx.wait() {
                Ok(tx_channel) => {
                    self.tx_channel = Some(tx_channel);
                    Ok(())
                }
                Err((err, tx_channel)) => {
                    self.tx_channel = Some(tx_channel);
                    Err(crate::Error::Rmt(err))
                }
            }
        } else {
            self.in_flight = Some(tx);
            Ok(())
        }
    }
}
