use crate::bus::Bus;
use crate::register::pwr_ctrl::PowerMode;
use crate::typestate::sample::Sample;
use crate::{Bmp390, Bmp390Result, Interrupts};
use core::marker::PhantomData;
use embedded_hal::digital::InputPin;
use embedded_hal_async::digital::Wait;
use crate::register::int_status::IntStatus;

pub struct ForcedDevice<Out, B, IrqPin> {
    device: Bmp390<B>,
    irq_pin: Option<IrqPin>,
    _phantom_data: PhantomData<Out>,
}

/*impl<B: Bus, IrqPin: Wait + InputPin> ForcedDevice<Pressure, B, IrqPin> {
    async fn read_value_if_irq(&mut self) -> Bmp390Result<Option<Sample<Pressure>>, B::Error> {
        let status = self.device.read::<Status>().await?;
        if status.drdy_press {
            let data = self.device.read_sensor_data().await?;

            Ok(Some(Sample::new(0.0, data.pressure)))
        }
        else {
            Ok(None)
        }
    }
}*/

impl<Out, B: Bus, IrqPin: Wait + InputPin> ForcedDevice<Out, B, IrqPin> {
    pub(crate) async fn new(mut device: Bmp390<B>, irq_pin: Option<IrqPin>) -> Bmp390Result<ForcedDevice<Out, B, IrqPin>, B::Error> {
        if irq_pin.is_some() {
            device.mask_interrupts(Interrupts::new().fifo_full().fifo_watermark()).await?;
        }
        Ok(Self {
            device,
            irq_pin,
            _phantom_data: PhantomData,
        })
    }
    
    pub async fn read_sample(&mut self) -> Bmp390Result<Sample<Out>, B::Error> {
        self.device.set_mode(PowerMode::Forced).await?;

        if let Some(irq_pin) = &mut self.irq_pin {
            // TODO Remove unwrap here later
            loop {
                while irq_pin.is_high().unwrap() {
                    let int_status = self.device.read::<IntStatus>().await?;
                    if int_status.drdy {
                        let data = self.device.read_sensor_data().await?;
                        return Ok(Sample::new(data.temperature, data.pressure));
                    }
                }

                // TODO Remove unwrap here later
                irq_pin.wait_for_rising_edge().await.unwrap();
            }

        }

        Ok(Sample::new(0.0, 0.0))
        /*
         There are multiple options for waiting for data to be ready:
         1. Wait for the maximum measurement time as specified by the datasheet section 3.9.1. The datasheet mentions this in section 3.10:
            "The timing for data readout in forced mode should be done so that the maximum measurement times are respected."
         2. Poll drdy_press / drdy_temp from the Status register.
         3. Wait for interrupts.
         */

    }
}
