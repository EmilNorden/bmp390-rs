use crate::{register, Bmp390Error, Bmp390Result};
use crate::bus::Bus;

pub struct CalibrationData {
    par_t1: f32,
    par_t2: f32,
    par_t3: f32,
    par_p1: f32,
    par_p2: f32,
    par_p3: f32,
    par_p4: f32,
    par_p5: f32,
    par_p6: f32,
    par_p7: f32,
    par_p8: f32,
    par_p9: f32,
    par_p10: f32,
    par_p11: f32,
    t_lin: f32,
}

impl CalibrationData {
    pub async fn new<B: Bus>(bus: &mut B) -> Bmp390Result<Self, B::Error> {
        let mut buf = [0u8; 21];
        let calib_coeffs = bus.read::<register::Calibration>().await?;


        Ok(Self {
            par_t1: (calib_coeffs.nvm_par_t1 as f32) / 0.00390625,
            par_t2: (calib_coeffs.nvm_par_t2 as f32) / 1073741824.0,
            par_t3: (calib_coeffs.nvm_par_t3 as f32) / 281474976710656.0,
            par_p1: (calib_coeffs.nvm_par_p1 as f32 - 16384.0) / 1048576.0,
            par_p2: (calib_coeffs.nvm_par_p2 as f32 - 16384.0) / 536870912.0,
            par_p3: (calib_coeffs.nvm_par_p3 as f32) / 4294967296.0,
            par_p4: (calib_coeffs.nvm_par_p4 as f32) / 137438953472.0,
            par_p5: (calib_coeffs.nvm_par_p5 as f32) / 0.125,
            par_p6: (calib_coeffs.nvm_par_p6 as f32) / 64.0,
            par_p7: (calib_coeffs.nvm_par_p7 as f32) / 256.0,
            par_p8: (calib_coeffs.nvm_par_p8 as f32) / 32768.0,
            par_p9: (calib_coeffs.nvm_par_p9 as f32) / 281474976710656.0,
            par_p10: (calib_coeffs.nvm_par_p10 as f32) / 281474976710656.0,
            par_p11: (calib_coeffs.nvm_par_p11 as f32) / 36893488147419103232.0,
            t_lin: 0.0,
        })
    }

    pub fn compensate_temperature(&mut self, temp: u32) -> f32 {
        let partial_data1 = temp as f32 - self.par_t1;
        let partial_data2 = partial_data1 * self.par_t2;

        self.t_lin = partial_data2 + (partial_data1 * partial_data1) * self.par_t3;

        self.t_lin
    }

    pub fn compensate_pressure(&self, pressure: u32) -> f32 {
        let pressure = pressure as f32;
        let t_lin = self.t_lin;
        let partial_data1 = self.par_p6 * t_lin;
        let partial_data2 = self.par_p7 * (t_lin * t_lin);
        let partial_data3 = self.par_p8 * (t_lin * t_lin * t_lin);
        let partial_out1 = self.par_p5 + partial_data1 + partial_data2 + partial_data3;

        let partial_data1 = self.par_p2 * t_lin;
        let partial_data2 = self.par_p3 * (t_lin * t_lin);
        let partial_data3 = self.par_p4 * (t_lin * t_lin * t_lin);
        let partial_out2 = pressure * (self.par_p1 + partial_data1 + partial_data2 + partial_data3);

        let partial_data1 = pressure * pressure;
        let partial_data2 = self.par_p9 + self.par_p10 * t_lin;
        let partial_data3 = partial_data1 * partial_data2;
        let partial_data4 = partial_data3 + (pressure * pressure * pressure) * self.par_p11;
        partial_out1 + partial_out2 + partial_data4
    }
}

#[inline]
const fn pow2f(e: i32) -> f32 {
    match e {
        // Normal numbers: exponent E = e + 127, mantissa = 0
        -126..=127 => f32::from_bits(((e + 127) as u32) << 23),

        // Subnormals: exponent bits = 0, value = F * 2^-149
        // Choose F = 1 << (e + 149) so value = 2^e
        -149..=-127 => f32::from_bits(1u32 << (e + 149)),

        // Underflow / overflow
        e if e < -149 => 0.0,
        _ => f32::INFINITY,
    }
}

#[cfg(test)]
mod tests {
    use smallvec::SmallVec;
    use crate::bus::Bus;
    use crate::register::{Readable, Writable};
    use super::*;

    struct FakeBus<'a> {
        pub data: &'a [u8],
        pub read_bytes: usize,
        // TODO Replace this field
        // pub read_registers: SmallVec<[Register; 10]>,
    }

    impl<'a> FakeBus<'a> {
        pub fn new(data: &'a[u8]) -> Self {
            FakeBus {
                data,
                read_bytes: 0,
                //read_registers: SmallVec::new(),
            }
        }
    }

    impl Bus for FakeBus<'_> {
        type Error = ();
/*
        async fn write_register(&mut self, reg: Register, data: u8) -> Result<(), Bmp390Error<Self::Error>> {
            todo!()
        }

        async fn read_register(&mut self, reg: Register, data: &mut [u8]) -> Result<(), Bmp390Error<Self::Error>> {
            self.read_registers.push(reg);
            data.copy_from_slice(&self.data[self.read_bytes..self.read_bytes + data.len()]);
            self.read_bytes += data.len();

            Ok(())
        }
*/
        async fn read<R: Readable>(&mut self) -> Result<R::Out, Bmp390Error<Self::Error>> {
            todo!()
        }

        async fn write<W: Writable>(&mut self, v: &W::In) -> Result<(), Bmp390Error<Self::Error>> {
            todo!()
        }
    }
    
    #[tokio::test]
    async fn test_load_calibration() {
        let mut bus = FakeBus::new(&[
            0x12, 0x34, 0x56, 0x78, 0x1, 0xA, 0x1, 0xAB, 0xCD, 0x42, 0xFF, 0xEF, 0xBE, 0xAD, 0xB, 0x2, 0xE7, 0xFE, 0xFE, 0x80, 0x40
        ]);

        let cb = CalibrationData::new(&mut bus).await.unwrap();

        //assert_eq!(bus.read_registers.len(), 1);
        //assert_eq!(bus.read_registers[0], Register::CalibrationDataStart);

        assert_eq!(cb.par_t1, 3412480.0);
        assert_eq!(cb.par_t2, 2.8690323e-5);
        assert_eq!(cb.par_t3, 3.5527137e-15);
        assert_eq!(cb.par_p1, -0.015371323);
        assert_eq!(cb.par_p2, -5.451776e-5);
        assert_eq!(cb.par_p3, 1.53668225e-8);
        assert_eq!(cb.par_p4, -0.000000000007275958);
        assert_eq!(cb.par_p5, 391032.0);
        assert_eq!(cb.par_p6, 46.703125);
        assert_eq!(cb.par_p7, 0.0078125);
        assert_eq!(cb.par_p8, -0.00076293945);
        assert_eq!(cb.par_p9, -0.0000000000009166001);
        assert_eq!(cb.par_p10, -0.00000000000045474735);
        assert_eq!(cb.par_p11, 0.0000000000000000017347235);
    }
}