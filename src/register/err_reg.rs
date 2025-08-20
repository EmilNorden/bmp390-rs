use crate::Bmp390;
use crate::register::{InvalidRegisterField, Readable, Reg};

/// Marker struct for the ERR_REG (0x02) register
///
/// - **Length:** 1 byte
/// - **Access:** Read-only
///
/// Used with [`Bmp390::read::<ErrReg>()`] or the convenience method
/// [`Bmp390::error_flags`].
pub struct ErrReg;
impl Reg for ErrReg {  const ADDR:u8 = 0x02; }

pub struct ErrorFlags {
    fatal_err: bool,
    cmd_err: bool,
    conf_err: bool,
}

impl ErrorFlags {
    pub fn new(fatal_err: bool, cmd_err: bool, conf_err: bool) -> Self {
        Self { fatal_err, cmd_err, conf_err }
    }

    /// A fatal error occurred.
    pub fn fatal_error(&self) -> bool { self.fatal_err }

    /// Command execution failed.
    ///
    /// This value is cleared on **register** read.
    pub fn command_error(&self) -> bool { self.cmd_err }

    /// Sensor configuration error detected.
    ///
    /// This can only happen in [`Normal´] power mode.
    /// This value is cleared on **register** read.
    pub fn configuration_error(&self) -> bool { self.conf_err }
}

impl Readable for ErrReg {
    type Out = ErrorFlags;

    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField> {
        Ok(ErrorFlags {
            fatal_err: (b[0] & 0b001) != 0,
            cmd_err: (b[0] & 0b010) != 0,
            conf_err: (b[0] & 0b100) != 0,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn err_reg_decode() {
        let reg = ErrReg::decode(&[0b001]).unwrap();
        assert_eq!([true, false, false], [reg.fatal_err, reg.cmd_err, reg.conf_err]);

        let reg = ErrReg::decode(&[0b010]).unwrap();
        assert_eq!([false, true, false], [reg.fatal_err, reg.cmd_err, reg.conf_err]);

        let reg = ErrReg::decode(&[0b100]).unwrap();
        assert_eq!([false, false, true], [reg.fatal_err, reg.cmd_err, reg.conf_err]);
    }
}