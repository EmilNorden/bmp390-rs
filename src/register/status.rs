use crate::register::{InvalidRegisterField, Readable, Reg};

/// Marker struct for the STATUS (0x03) register
///
/// - **Length:** 1 byte
/// - **Access:** Read-only
///
/// Used with [`Bmp390::read::<Status>()`] or the convenience method
/// [`Bmp390::status`].
pub struct Status;

impl Reg for Status { const ADDR: u8 = 0x03; }

#[derive(Copy, Clone, Debug)]
pub struct StatusFlags {
    /// Is the command decoder ready to accept a new command?
    ///
    /// [`false`] means that a command is already in progress.
    pub cmd_rdy: bool,

    /// Is there new pressure data to be read?
    ///
    /// This value is cleared when any of the pressure DATA registers are read.
    pub drdy_press: bool,

    /// Is there new temperature data to be read?
    ///
    /// This value is cleared when any of the temperature DATA registers are read.
    pub drdy_temp: bool,
}

impl Readable for Status {
    type Out = StatusFlags;
    fn decode(b: &[u8]) -> Result<Self::Out, InvalidRegisterField> {
        Ok(StatusFlags {
            cmd_rdy:    (b[0] & 0b0010000) != 0,
            drdy_press: (b[0] & 0b0100000) != 0,
            drdy_temp:  (b[0] & 0b1000000) != 0,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn status_decode() {
        let reg = Status::decode(&[0b0010000]).unwrap();
        assert_eq!([true, false, false], [reg.cmd_rdy, reg.drdy_press, reg.drdy_temp]);

        let reg = Status::decode(&[0b0100000]).unwrap();
        assert_eq!([false, true, false], [reg.cmd_rdy, reg.drdy_press, reg.drdy_temp]);

        let reg = Status::decode(&[0b1000000]).unwrap();
        assert_eq!([false, false, true], [reg.cmd_rdy, reg.drdy_press, reg.drdy_temp]);
    }
}