//! ### INT_CTRL - Interrupt control (`0x19`, 1 byte, R/W)
//!
//! Configures interrupts, the INT pin, and `INT_STATUS` register.
//!
//! ### Default values
//! 0x02 (INT pin active high)
//!
//! ### Examples
//! ```rust,no_run
//! # use crate::bmp390_rs::{Bmp390, Bmp390Result};
//! # use crate::bmp390_rs::bus::Bus;
//! # async fn demo<B: Bus>(mut device: Bmp390<B>)
//! #     -> Bmp390Result<(), B::Error> {
//! use bmp390_rs::register::int_ctrl::{IntCtrl, IntCtrlCfg};
//!
//! // Write new interrupt configuration.
//! device.write::<IntCtrl>(&IntCtrlCfg {
//!     int_od: false,      // Keep INT pin push-pull
//!     int_level: true,    // INT pin should be active HIGH
//!     int_latch: false,   // Do not latch interrupts
//!     fwtm_en: true,      // Enable FIFO watermark interrupt
//!     ffull_en: true,     // Enable FIFO full
//!     int_ds: false,      // Drive strength LOW
//!     drdy_en: true       // Enable data ready interrupt
//! }).await?;
//!
//! # Ok(()) }
//! ```
use crate::register::{InvalidRegisterField, Readable, Reg, Writable};

/// Marker type for INT_CTRL (0x19) register
///
/// - **Length:** 1 byte
/// - **Access:** Read/Write
///
/// Used with [`Bmp390::read::<IntCtrl>()`] or [`Bmp390::write::<IntCtrl>()`]
pub struct IntCtrl;
impl Reg for IntCtrl { const ADDR: u8 = 0x19; }

/// The payload for the INT_CTRL (0x19) register.
#[derive(Copy, Clone, Debug)]
pub struct IntCtrlCfg {
    /// True if INT pin is open-drain, false for push-pull.
    pub int_od: bool,
    /// True if INT pin is active high, false for active low.
    pub int_level: bool,
    /// True if interrupts for INT pin and INT_STATUS register should be latched.
    pub int_latch: bool,
    /// Enable/disable FIFO watermark interrupt.
    ///
    /// This interrupt is asserted when the FIFO reaches a specific length. This length is configurable in the [`FifoWtm`] register.
    pub fwtm_en: bool,
    /// Enable/disable FIFO full interrupt.
    ///
    /// This interrupt is asserted when the FIFO is full (FIFO length is >= 504).
    pub ffull_en: bool,
    /// Drive strength of the INT pin (?)
    ///
    /// It is not clear from the datasheet what this field is. But some investigation suggests that it is drive strength for the INT pin.
    /// The levels are not provided in the datasheet, but false is *low* drive strength and true is *high* drive strength.
    pub int_ds: bool,
    /// Enable/disable data ready interrupt.
    ///
    /// This interrupt is asserted when a new measurement has been performed and stored in the data registers.
    pub drdy_en: bool,
}

impl Readable for IntCtrl {
    type Out = IntCtrlCfg;

    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField> {
        Ok(IntCtrlCfg {
            int_od:     (b[0] & 0b0000_0001) != 0,
            int_level:  (b[0] & 0b0000_0010) != 0,
            int_latch:  (b[0] & 0b0000_0100) != 0,
            fwtm_en:    (b[0] & 0b0000_1000) != 0,
            ffull_en:   (b[0] & 0b0001_0000) != 0,
            int_ds:     (b[0] & 0b0010_0000) != 0,
            drdy_en:    (b[0] & 0b0100_0000) != 0,
        })
    }
}

impl Writable for IntCtrl {
    type In = IntCtrlCfg;

    fn encode(v: &Self::In, out: &mut [u8]) {
        let mut value = 0u8;
        if v.int_od { value |= 0b0000_0001; }
        if v.int_level { value |= 0b0000_0010;}
        if v.int_latch { value |= 0b0000_0100; }
        if v.fwtm_en { value |= 0b0000_1000; }
        if v.ffull_en { value |= 0b0001_0000; }
        if v.int_ds { value |= 0b0010_0000; }
        if v.drdy_en { value |= 0b0100_0000; }

        out[0] = value;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn int_ctrl_decode() {
        let reg = IntCtrl::decode(&[0b0000_0001]).unwrap();
        assert!(reg.int_od);

        let reg = IntCtrl::decode(&[0b0000_0010]).unwrap();
        assert!(reg.int_level);

        let reg = IntCtrl::decode(&[0b0000_0100]).unwrap();
        assert!(reg.int_latch);

        let reg = IntCtrl::decode(&[0b0000_1000]).unwrap();
        assert!(reg.fwtm_en);

        let reg = IntCtrl::decode(&[0b0001_0000]).unwrap();
        assert!(reg.ffull_en);

        let reg = IntCtrl::decode(&[0b0010_0000]).unwrap();
        assert!(reg.int_ds);

        let reg = IntCtrl::decode(&[0b0100_0000]).unwrap();
        assert!(reg.drdy_en);
    }

    #[test]
    fn int_ctrl_encode() {
        let mut buffer = [0u8; 1];
        IntCtrl::encode(&IntCtrlCfg {
            int_od: true,
            int_level: false,
            int_latch: false,
            fwtm_en: false,
            ffull_en: false,
            int_ds: false,
            drdy_en: false,
        }, &mut buffer);
        assert_eq!([0b0000_0001], buffer);

        IntCtrl::encode(&IntCtrlCfg {
            int_od: false,
            int_level: true,
            int_latch: false,
            fwtm_en: false,
            ffull_en: false,
            int_ds: false,
            drdy_en: false,
        }, &mut buffer);
        assert_eq!([0b0000_0010], buffer);

        IntCtrl::encode(&IntCtrlCfg {
            int_od: false,
            int_level: false,
            int_latch: true,
            fwtm_en: false,
            ffull_en: false,
            int_ds: false,
            drdy_en: false,
        }, &mut buffer);
        assert_eq!([0b0000_0100], buffer);

        IntCtrl::encode(&IntCtrlCfg {
            int_od: false,
            int_level: false,
            int_latch: false,
            fwtm_en: true,
            ffull_en: false,
            int_ds: false,
            drdy_en: false,
        }, &mut buffer);
        assert_eq!([0b0000_1000], buffer);

        IntCtrl::encode(&IntCtrlCfg {
            int_od: false,
            int_level: false,
            int_latch: false,
            fwtm_en: false,
            ffull_en: true,
            int_ds: false,
            drdy_en: false,
        }, &mut buffer);
        assert_eq!([0b0001_0000], buffer);

        IntCtrl::encode(&IntCtrlCfg {
            int_od: false,
            int_level: false,
            int_latch: false,
            fwtm_en: false,
            ffull_en: false,
            int_ds: true,
            drdy_en: false,
        }, &mut buffer);
        assert_eq!([0b0010_0000], buffer);

        IntCtrl::encode(&IntCtrlCfg {
            int_od: false,
            int_level: false,
            int_latch: false,
            fwtm_en: false,
            ffull_en: false,
            int_ds: false,
            drdy_en: true,
        }, &mut buffer);
        assert_eq!([0b0100_0000], buffer);
    }
}