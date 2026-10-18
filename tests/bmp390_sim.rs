//! Deterministic simulation of the BMP390 driver against a model of the sensor.
//!
//! The model follows the BMP390 datasheet (BST-BMP390-DS002, revision 1.7):
//! the register map and the reset values (table 25), the power modes and the
//! legal mode transitions (section 3.3), the measurement time (section 3.9),
//! the configuration error in normal mode (register ERR_REG), the status and
//! clear-on-read flags, the FIFO frames, the overflow and the partial reads
//! (section 3.6), the INT pin (section 3.7), the soft reset and the SPI and
//! I2C protocols (chapter 5). It sits under the `SpiDevice`, `I2c`, `InputPin`
//! and `Wait` traits, so the bus wrappers of the driver run too. A seeded
//! random sequence of calls runs against the model, with random pauses, random
//! gaps between bus transactions (other tasks of an async executor) and bus
//! faults: calls of `Bmp390` over SPI and I2C, and of each typestate variant.
//! At the start the sensor is either fresh from power-on or still configured
//! by an earlier run of the firmware.
//!
//! The model panics when a transaction on the bus breaks the datasheet. After
//! each call, the test compares the result with what the model did. The
//! decoders and the compensation formulas in this file follow the datasheet
//! and do not share code with the driver. Counters prove that the runs reach
//! every call, every error and the races between the bus transactions.
//!
//! Reproduce a failure: `BMP390_SIM_SEED=<seed> cargo test --test bmp390_sim`;
//! `BMP390_SIM_TRACE=1` prints every call.

use core::cell::RefCell;
use core::future::Future;
use core::pin::pin;
use core::task::{Context, Poll, Waker};
use std::collections::VecDeque;

use bmp390_rs::bus::Bus;
use bmp390_rs::config::Configuration;
use bmp390_rs::error::Bmp390Error;
use bmp390_rs::fifo::{
    ControlFrameType, FifoConfiguration, FifoFrame, FifoFullBehavior, SensorFrameType,
};
use bmp390_rs::register::cmd::{Cmd, CmdData};
use bmp390_rs::register::config::IIRFilterCoefficient;
use bmp390_rs::register::fifo_wtm::FifoWtm;
use bmp390_rs::register::odr::OutputDataRate;
use bmp390_rs::register::osr::{OsrCfg, Oversampling};
use bmp390_rs::register::pwr_ctrl::PowerMode;
use bmp390_rs::{Bmp390, Interrupts, ResetPolicy, SdoPinState};
use embedded_hal_async::delay::DelayNs;

const SEEDS: u64 = 200;
const STEPS: usize = 400;

const BMP390_CHIP_ID: u8 = 0x60;
const BMP388_CHIP_ID: u8 = 0x50;

const FIFO_CAPACITY: usize = 512;
/// "FIFO full means free space is less than maximum frame length (9 bytes)" (section 3.6.4)
const MAX_FRAME: usize = 9;
/// Fill level of the FIFO full interrupt (section 3.7.5.2)
const FIFO_FULL_LEVEL: usize = 504;

/// The datasheet gives no duration for the soft reset. The start-up time to
/// the first communication is 2 ms (table 2), and the Bosch reference driver
/// waits 2 ms after the soft reset command.
const SOFT_RESET_US: u64 = 2_000;

/// The largest ratio of the maximum to the typical measurement time in table 23
/// (5.70 ms / 4.82 ms), in units of 1/1000
const MAX_TIME_PER_MILLE: u64 = 1_183;

/// xorshift64*: deterministic and without dependencies
struct Rng(u64);

impl Rng {
    fn new(seed: u64) -> Self {
        Self(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1)
    }

    fn next(&mut self) -> u64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    fn below(&mut self, n: u64) -> u64 {
        self.next() % n
    }

    fn one_in(&mut self, n: u64) -> bool {
        self.below(n) == 0
    }

    fn byte(&mut self) -> u8 {
        self.next() as u8
    }
}

/// Typical measurement time in µs (section 3.9.2)
fn conversion_us(press_en: bool, temp_en: bool, osr_p: u8, osr_t: u8) -> u64 {
    234 + u64::from(press_en) * (392 + (1 << osr_p) * 2020)
        + u64::from(temp_en) * (163 + (1 << osr_t) * 2020)
}

fn max_conversion_us(press_en: bool, temp_en: bool, osr_p: u8, osr_t: u8) -> u64 {
    (conversion_us(press_en, temp_en, osr_p, osr_t) * MAX_TIME_PER_MILLE).div_ceil(1000)
}

/// Sampling period in µs: 5 ms times the prescaler 2^odr_sel; values above 17
/// saturate at 17 (tables 44 and 45)
fn period_us(odr_sel: u8) -> u64 {
    5_000 << odr_sel.min(17)
}

/// Normal mode needs the measurement to fit into the sampling period
/// (section 3.9.2: odr_sel >= log2(200 Hz / f_max))
fn fits_normal_mode(press_en: bool, temp_en: bool, osr_p: u8, osr_t: u8, odr_sel: u8) -> bool {
    conversion_us(press_en, temp_en, osr_p, osr_t) < period_us(odr_sel)
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Mode {
    Sleep,
    Forced,
    Normal,
}

impl Mode {
    /// mode[1:0] in PWR_CTRL (table 42)
    fn from_bits(bits: u8) -> Self {
        match bits & 0b11 {
            0b00 => Mode::Sleep,
            0b01 | 0b10 => Mode::Forced,
            _ => Mode::Normal,
        }
    }

    fn bits(self) -> u8 {
        match self {
            Mode::Sleep => 0b00,
            Mode::Forced => 0b01,
            Mode::Normal => 0b11,
        }
    }
}

/// A measurement in progress
#[derive(Clone, Copy, Debug)]
struct Running {
    id: u64,
    start_us: u64,
    end_us: u64,
    press_en: bool,
    temp_en: bool,
}

/// What the sensor did during one driver call
#[derive(Default, Debug)]
struct Served {
    fault: bool,
    /// A fault after the chip id was read; the probe of `new` retries faults before
    fault_after_probe: bool,
    /// Every read burst: the start address and the data bytes
    bursts: Vec<(u8, Vec<u8>)>,
    /// Every register write: address and value
    writes: Vec<(u8, u8)>,
    /// Stored FIFO frames that a burst read completely
    consumed: Vec<Vec<u8>>,
    /// Bytes of every FIFO_DATA burst
    fifo_bursts: Vec<Vec<u8>>,
    conf_errors: usize,
    /// Measurements that a write to PWR_CTRL started
    started: Vec<(Mode, u64)>,
    /// Mode commands that the sensor ignored
    ignored_modes: Vec<Mode>,
    soft_resets: usize,
    /// Measurements behind the pressure and the temperature data at the last data read
    data_ids: Option<(u64, u64)>,
    /// The call wrote sleep mode to PWR_CTRL
    sleep_written: bool,
    /// Reads of PWR_CTRL after that write that found the device not yet asleep
    not_asleep_reads: usize,
}

impl Served {
    fn last_burst(&self, addr: u8) -> &[u8] {
        &self
            .bursts
            .iter()
            .rev()
            .find(|(a, _)| *a == addr)
            .unwrap_or_else(|| panic!("no read of 0x{addr:02x}"))
            .1
    }
}

/// State of the current FIFO_DATA burst
#[derive(Default)]
struct FifoBurst {
    active: bool,
    bytes: Vec<u8>,
    /// Bytes of the overread frame (sensor time or empty) and the position in it
    overread: Option<(Vec<u8>, usize)>,
    /// The sensor time frame was sent in this burst
    time_sent: bool,
}

struct Model {
    rng: Rng,
    faults: bool,
    now_us: u64,
    chip_id: u8,
    nvm: [u8; 21],
    busy_until_us: u64,

    // Registers (table 25)
    int_ctrl: u8,
    if_conf: u8,
    /// press_en and temp_en of PWR_CTRL; the mode bits come from `mode`
    enables: u8,
    osr: u8,
    odr: u8,
    config: u8,
    fifo_wtm: u16,
    fifo_config_1: u8,
    fifo_config_2: u8,
    data_press: u32,
    data_temp: u32,

    // Flags
    fatal_err: bool,
    cmd_err: bool,
    conf_err: bool,
    drdy_press: bool,
    drdy_temp: bool,
    por_detected: bool,
    itf_act_pt: bool,
    int_drdy: bool,
    int_fwm: bool,
    int_ffull: bool,
    /// End of the data ready pulse on the INT pin in non-latched mode
    drdy_pulse_until: Option<u64>,
    /// A measurement uses the oversampling of its start
    osr_written_us: u64,

    // Measurement engine
    mode: Mode,
    /// A mode command waits for the end of the running measurement (section 3.3.4)
    pending_mode: Option<Mode>,
    running: Option<Running>,
    next_start_us: u64,
    /// Last transition from sleep to normal mode
    normal_since_us: Option<u64>,
    measurements: u64,
    /// Measurement that wrote each data register
    press_id: u64,
    temp_id: u64,

    // FIFO
    fifo: VecDeque<Vec<u8>>,
    /// Bytes of the oldest frame already sent in the current burst
    head_read: usize,
    fifo_config_changed: bool,
    subsample_count: u64,
    burst: FifoBurst,

    served: Served,
    spi: bool,
    i2c_address: u8,
    i2c_pointer: u8,
}

impl Model {
    fn new(seed: u64, spi: bool) -> Self {
        let mut rng = Rng::new(seed);
        let chip_id = match rng.below(20) {
            0 => 0x58, // a BMP280: the driver must not accept it
            1..=3 => BMP388_CHIP_ID,
            _ => BMP390_CHIP_ID,
        };
        let nvm = if rng.one_in(2) {
            // Coefficients of a real sensor
            [
                0x98, 0x6E, 0x13, 0x4D, 0xF9, 0xB0, 0x1B, 0xC0, 0x15, 0x06, 0x01, 0x92, 0x4A, 0xAE,
                0x5D, 0x03, 0xFA, 0x08, 0x0F, 0x06, 0xF5,
            ]
        } else {
            core::array::from_fn(|_| rng.byte())
        };
        let i2c_address = if rng.one_in(2) { 0x76 } else { 0x77 };
        let mut model = Self {
            rng,
            faults: true,
            now_us: 0,
            chip_id,
            nvm,
            busy_until_us: 0,
            int_ctrl: 0,
            if_conf: 0,
            enables: 0,
            osr: 0,
            odr: 0,
            config: 0,
            fifo_wtm: 0,
            fifo_config_1: 0,
            fifo_config_2: 0,
            data_press: 0,
            data_temp: 0,
            fatal_err: false,
            cmd_err: false,
            conf_err: false,
            drdy_press: false,
            drdy_temp: false,
            por_detected: false,
            itf_act_pt: false,
            int_drdy: false,
            int_fwm: false,
            int_ffull: false,
            drdy_pulse_until: None,
            osr_written_us: 0,
            mode: Mode::Sleep,
            pending_mode: None,
            running: None,
            next_start_us: 0,
            normal_since_us: None,
            measurements: 0,
            press_id: 0,
            temp_id: 0,
            fifo: VecDeque::new(),
            head_read: 0,
            fifo_config_changed: false,
            subsample_count: 0,
            burst: FifoBurst::default(),
            served: Served::default(),
            spi,
            i2c_address,
            i2c_pointer: 0,
        };
        model.power_on_reset();
        if model.rng.one_in(2) {
            model.previous_life();
        }
        model
    }

    /// Reset values of table 25; after power-on the sensor is in sleep mode (section 3.2)
    fn power_on_reset(&mut self) {
        self.int_ctrl = 0x02;
        self.if_conf = 0x00;
        self.enables = 0x00;
        self.osr = 0x02;
        self.odr = 0x00;
        self.config = 0x00;
        self.fifo_wtm = 0x001;
        self.fifo_config_1 = 0x02;
        self.fifo_config_2 = 0x02;
        self.data_press = 0x80_0000;
        self.data_temp = 0x80_0000;
        self.cmd_err = false;
        self.conf_err = false;
        self.drdy_press = false;
        self.drdy_temp = false;
        self.por_detected = true;
        self.itf_act_pt = false;
        self.int_drdy = false;
        self.int_fwm = false;
        self.int_ffull = false;
        self.drdy_pulse_until = None;
        self.mode = Mode::Sleep;
        self.pending_mode = None;
        self.running = None;
        self.normal_since_us = None;
        self.fifo.clear();
        self.head_read = 0;
        self.fifo_config_changed = false;
    }

    /// The MCU restarted, the sensor kept its power: a configuration from the
    /// previous firmware run, often still in normal mode
    fn previous_life(&mut self) {
        self.enables = self.rng.below(4) as u8;
        self.osr = (self.rng.below(6) | self.rng.below(6) << 3) as u8;
        self.odr = self.rng.below(18) as u8;
        self.config = (self.rng.below(8) as u8) << 1;
        self.fifo_config_1 = self.rng.below(32) as u8;
        self.fifo_config_2 = self.rng.below(32) as u8 & 0b0000_1111;
        self.int_ctrl = self.rng.below(128) as u8;
        self.por_detected = false;
        let (press_en, temp_en, osr_p, osr_t) = self.measurement_settings();
        if self.rng.one_in(3) && fits_normal_mode(press_en, temp_en, osr_p, osr_t, self.odr) {
            self.mode = Mode::Normal;
            self.next_start_us = self.rng.below(period_us(self.odr));
        }
    }

    fn measurement_settings(&self) -> (bool, bool, u8, u8) {
        (
            self.enables & 0b01 != 0,
            self.enables & 0b10 != 0,
            self.osr & 0b111,
            (self.osr >> 3) & 0b111,
        )
    }

    fn fits_normal_mode(&self) -> bool {
        let (press_en, temp_en, osr_p, osr_t) = self.measurement_settings();
        fits_normal_mode(press_en, temp_en, osr_p, osr_t, self.odr)
    }

    fn fifo_len(&self) -> usize {
        self.fifo.iter().map(Vec::len).sum()
    }

    /// FIFO writes need fifo_mode and at least one channel (section 3.6)
    fn fifo_enabled(&self) -> bool {
        self.fifo_config_1 & 0b1 != 0 && self.fifo_config_1 & 0b1_1000 != 0
    }

    fn sensor_time(&self) -> u32 {
        // The datasheet does not give the tick; any 24-bit counter works here
        ((self.now_us * 10 / 390) & 0xFF_FFFF) as u32
    }

    /// Runs the sensor until `t_us`
    fn advance_to(&mut self, t_us: u64) {
        loop {
            let end = self.running.map(|r| r.end_us);
            let start =
                (self.running.is_none() && self.mode == Mode::Normal).then_some(self.next_start_us);
            match (end, start) {
                (Some(end), _) if end <= t_us => {
                    self.now_us = self.now_us.max(end);
                    self.finish_measurement();
                }
                (None, Some(start)) if start <= t_us => {
                    self.now_us = self.now_us.max(start);
                    self.start_measurement(Mode::Normal);
                }
                _ => break,
            }
        }
        self.now_us = self.now_us.max(t_us);
    }

    fn start_measurement(&mut self, mode: Mode) {
        let (press_en, temp_en, osr_p, osr_t) = self.measurement_settings();
        let typical = conversion_us(press_en, temp_en, osr_p, osr_t);
        let duration = if mode == Mode::Normal {
            if !self.fits_normal_mode() {
                // ERR_REG.conf_err: "sensor configuration error detected (only
                // working in normal mode)", and a control frame in the FIFO
                self.conf_err = true;
                self.served.conf_errors += 1;
                if self.fifo_enabled() {
                    self.push_frame(vec![0x44, 0x01]);
                }
            }
            typical
        } else {
            let max = max_conversion_us(press_en, temp_en, osr_p, osr_t);
            typical + self.rng.below(max - typical + 1)
        };
        self.measurements += 1;
        self.running = Some(Running {
            id: self.measurements,
            start_us: self.now_us,
            end_us: self.now_us + duration,
            press_en,
            temp_en,
        });
    }

    fn finish_measurement(&mut self) {
        let run = self.running.take().unwrap();
        // Raw values of the ADC that keep the compensated values in a sane range
        // for the coefficients of a real sensor
        if run.press_en {
            self.data_press = 5_000_000 + self.rng.below(3_000_000) as u32;
            self.press_id = run.id;
            self.drdy_press = true;
        }
        if run.temp_en {
            self.data_temp = 5_500_000 + self.rng.below(6_000_000) as u32;
            self.temp_id = run.id;
            self.drdy_temp = true;
        }
        self.int_drdy = true;
        if self.int_ctrl & 0b100 == 0 {
            // Non-latched: "Data ready INT pad contribution is cleared automatically
            // 2.5 ms after the interrupt assertion" (section 3.7.5.3)
            self.drdy_pulse_until = Some(self.now_us + 2_500);
        }
        self.store_in_fifo(run);

        if self.mode == Mode::Forced {
            // "the sensor returns to sleep mode" (section 3.3.2)
            self.mode = Mode::Sleep;
        }
        if let Some(mode) = self.pending_mode.take() {
            self.execute_mode(mode);
        } else if self.mode == Mode::Normal {
            self.next_start_us = (run.start_us + period_us(self.odr)).max(run.end_us);
        }
    }

    fn store_in_fifo(&mut self, run: Running) {
        if !self.fifo_enabled() {
            return;
        }
        if self.mode == Mode::Normal {
            // Down-sampling by 2^fifo_subsampling, normal mode only; the first
            // measurement after sleep -> normal is stored (section 3.6.2)
            let factor = 1 << (self.fifo_config_2 & 0b111);
            let store = self.subsample_count.is_multiple_of(factor);
            self.subsample_count += 1;
            if !store {
                return;
            }
            if self.fifo_config_changed {
                // Control frame "FIFO input configuration change" before the first
                // frame with the new configuration (section 3.6.5)
                self.fifo_config_changed = false;
                self.push_frame(vec![0x48, 0x01]);
            }
        }
        let p = run.press_en && self.fifo_config_1 & 0b0_1000 != 0;
        let t = run.temp_en && self.fifo_config_1 & 0b1_0000 != 0;
        if !p && !t {
            return;
        }
        // Table 13: header 10 s t 0 p 0 0; temperature before pressure (table 15)
        let mut frame = vec![0x80 | u8::from(t) << 4 | u8::from(p) << 2];
        if t {
            frame.extend_from_slice(&self.data_temp.to_le_bytes()[..3]);
        }
        if p {
            frame.extend_from_slice(&self.data_press.to_le_bytes()[..3]);
        }
        self.push_frame(frame);
    }

    fn push_frame(&mut self, frame: Vec<u8>) {
        let stop_on_full = self.fifo_config_1 & 0b10 != 0;
        if stop_on_full {
            // "The newest frame is discarded" (section 3.6.4)
            if FIFO_CAPACITY - self.fifo_len() < MAX_FRAME {
                return;
            }
        } else {
            // "FIFO will repeatedly delete the oldest frame until it creates enough space"
            while FIFO_CAPACITY - self.fifo_len() < frame.len() {
                self.fifo.pop_front();
                self.head_read = 0;
            }
        }
        self.fifo.push_back(frame);
        self.update_fifo_interrupts();
    }

    fn update_fifo_interrupts(&mut self) {
        let len = self.fifo_len();
        if len >= FIFO_FULL_LEVEL {
            self.int_ffull = true;
        }
        // "If the FIFO watermark level is set to zero, the interrupt condition
        // will never be satisfied" (section 3.7.5.1)
        if self.fifo_wtm != 0 && len >= usize::from(self.fifo_wtm) {
            self.int_fwm = true;
        }
    }

    /// Executes a legal mode command now
    fn execute_mode(&mut self, mode: Mode) {
        match (self.mode, mode) {
            (Mode::Sleep, Mode::Normal) => {
                self.mode = Mode::Normal;
                self.normal_since_us = Some(self.now_us);
                self.subsample_count = 0;
                self.fifo_config_changed = false;
                self.served.started.push((Mode::Normal, self.now_us));
                self.start_measurement(Mode::Normal);
            }
            (Mode::Sleep, Mode::Forced) => {
                self.mode = Mode::Forced;
                self.served.started.push((Mode::Forced, self.now_us));
                self.start_measurement(Mode::Forced);
            }
            (_, Mode::Sleep) => self.mode = Mode::Sleep,
            (from, to) => unreachable!("illegal transition {from:?} -> {to:?}"),
        }
    }

    /// A mode command in PWR_CTRL (section 3.3.4)
    fn mode_command(&mut self, mode: Mode) {
        if mode == self.mode && self.pending_mode.is_none() {
            // Not a transition: normal over normal does not restart the cycle
            return;
        }
        // Figure 4: sleep <-> normal, sleep -> forced; forced returns to sleep
        let legal = matches!(
            (self.mode, mode),
            (Mode::Sleep, Mode::Normal | Mode::Forced) | (Mode::Normal | Mode::Forced, Mode::Sleep)
        );
        if self.pending_mode.is_some() || !legal {
            // "Further mode change commands are ignored until the last mode change
            // command is executed. Also, mode change commands that are not legal
            // are ignored."
            self.served.ignored_modes.push(mode);
            return;
        }
        if self.running.is_some() {
            // "execution of mode switching commands is delayed until the end of
            // the currently running measurement period"
            self.pending_mode = Some(mode);
        } else {
            self.execute_mode(mode);
        }
    }

    // --- Bus side ---

    /// Other tasks of the executor may run before the driver continues
    fn gap(&mut self) {
        let gap = match self.rng.below(40) {
            0..=7 => self.rng.below(10_000),
            8 => self.rng.below(50_000),
            _ => 0,
        };
        self.advance_to(self.now_us + gap);
    }

    /// Level of the INT pin (sections 3.7.2 and 3.7.4)
    fn pin_high(&self) -> bool {
        let latched = self.int_ctrl & 0b100 != 0;
        let len = self.fifo_len();
        let drdy = if latched {
            self.int_drdy
        } else {
            self.drdy_pulse_until.is_some_and(|t| self.now_us < t)
        };
        let fwm = if latched {
            self.int_fwm
        } else {
            self.fifo_wtm != 0 && len >= usize::from(self.fifo_wtm)
        };
        let ffull = if latched {
            self.int_ffull
        } else {
            len >= FIFO_FULL_LEVEL
        };
        let active = self.int_ctrl & 0b0100_0000 != 0 && drdy
            || self.int_ctrl & 0b0000_1000 != 0 && fwm
            || self.int_ctrl & 0b0001_0000 != 0 && ffull;
        // int_level: 1 = active high, 0 = active low
        active == (self.int_ctrl & 0b10 != 0)
    }

    /// The next time at which the sensor state or the INT pin can change
    fn next_event_us(&self) -> Option<u64> {
        let end = self.running.map(|r| r.end_us);
        let start =
            (self.running.is_none() && self.mode == Mode::Normal).then_some(self.next_start_us);
        let pulse = self.drdy_pulse_until.filter(|&t| t > self.now_us);
        [end, start, pulse].into_iter().flatten().min()
    }

    /// Runs the sensor until the INT pin is at `high`, or changes to `high` if `edge`
    fn wait_for_pin(&mut self, high: bool, edge: bool) {
        self.gap();
        let mut was = self.pin_high();
        if !edge && was == high {
            return;
        }
        // Longer than the slowest sampling period (table 45)
        let deadline = self.now_us + 2 * period_us(17);
        loop {
            let t = self
                .next_event_us()
                .filter(|&t| t <= deadline)
                .unwrap_or_else(|| {
                    panic!(
                        "the driver waits for an INT {} that never comes: mode {:?}, INT_CTRL \
                     0x{:02x}, INT_STATUS drdy {}",
                        if edge { "edge" } else { "level" },
                        self.mode,
                        self.int_ctrl,
                        self.int_drdy
                    )
                });
            self.advance_to(t);
            let now = self.pin_high();
            if now == high && (was != high || !edge) {
                return;
            }
            was = now;
        }
    }

    /// Start of a bus transaction: other tasks may run before it, and the bus may fail
    fn begin(&mut self) -> Result<(), ()> {
        self.gap();
        assert!(
            self.now_us >= self.busy_until_us,
            "bus access {} us after the soft reset command; the reset takes {SOFT_RESET_US} us",
            SOFT_RESET_US - (self.busy_until_us - self.now_us)
        );
        if self.faults && self.rng.one_in(60) {
            self.served.fault = true;
            let chip_id = self.chip_id;
            self.served.fault_after_probe |= self
                .served
                .bursts
                .iter()
                .any(|(a, b)| *a == 0x00 && b[0] == chip_id);
            return Err(());
        }
        if self.running.is_some() {
            self.itf_act_pt = true;
        }
        Ok(())
    }

    /// End of a bus transaction (CSB rises or STOP)
    fn end(&mut self) {
        if self.burst.active {
            // A partly read frame is sent again on the next read (section 3.6.6.2)
            if self.head_read > 0 {
                self.head_read = 0;
            }
            let burst = core::mem::take(&mut self.burst);
            self.served.fifo_bursts.push(burst.bytes);
        }
    }

    fn read_burst(&mut self, addr: u8, buf: &mut [u8]) {
        if addr == 0x04 {
            self.served.data_ids = Some((self.press_id, self.temp_id));
        }
        let mut a = addr;
        for byte in buf.iter_mut() {
            *byte = self.read_register(a);
            // "the address counter stops incrementing when fifo_data address is reached"
            if a != 0x14 {
                a = a.wrapping_add(1) & 0x7F;
            }
        }
        self.served.bursts.push((addr, buf.to_vec()));
    }

    fn read_register(&mut self, addr: u8) -> u8 {
        match addr {
            0x00 => self.chip_id,
            0x01 => 0x01,
            0x02 => {
                let v = u8::from(self.fatal_err)
                    | u8::from(self.cmd_err) << 1
                    | u8::from(self.conf_err) << 2;
                // cmd_err and conf_err: "Cleared on read"
                self.cmd_err = false;
                self.conf_err = false;
                v
            }
            // cmd_rdy: the commands of this model execute at once
            0x03 => 1 << 4 | u8::from(self.drdy_press) << 5 | u8::from(self.drdy_temp) << 6,
            0x04..=0x06 => {
                // drdy_press "gets reset, when one pressure DATA register is read out"
                self.drdy_press = false;
                self.data_press.to_le_bytes()[usize::from(addr - 0x04)]
            }
            0x07..=0x09 => {
                self.drdy_temp = false;
                self.data_temp.to_le_bytes()[usize::from(addr - 0x07)]
            }
            0x0C..=0x0E => self.sensor_time().to_le_bytes()[usize::from(addr - 0x0C)],
            0x10 => {
                let v = u8::from(self.por_detected) | u8::from(self.itf_act_pt) << 1;
                self.por_detected = false;
                self.itf_act_pt = false;
                v
            }
            0x11 => {
                let v = u8::from(self.int_fwm)
                    | u8::from(self.int_ffull) << 1
                    | u8::from(self.int_drdy) << 3;
                // "cleared after reading"; a FIFO condition that still holds sets
                // its flag again
                self.int_fwm = false;
                self.int_ffull = false;
                self.int_drdy = false;
                // "For data ready, interrupt contribution is de-asserted by reading
                // int_status" (section 3.7.2)
                self.drdy_pulse_until = None;
                self.update_fifo_interrupts();
                v
            }
            0x12 => self.fifo_len().to_le_bytes()[0],
            0x13 => self.fifo_len().to_le_bytes()[1],
            0x14 => self.fifo_data_byte(),
            0x15 => self.fifo_wtm.to_le_bytes()[0],
            0x16 => self.fifo_wtm.to_le_bytes()[1],
            0x17 => self.fifo_config_1,
            0x18 => self.fifo_config_2,
            0x19 => self.int_ctrl,
            0x1A => self.if_conf,
            0x1B => {
                if self.served.sleep_written && self.mode != Mode::Sleep {
                    self.served.not_asleep_reads += 1;
                }
                self.enables | self.mode.bits() << 4
            }
            0x1C => self.osr,
            0x1D => self.odr,
            0x1F => self.config,
            0x31..=0x45 => self.nvm[usize::from(addr - 0x31)],
            // CMD: "Register will always read as 0x00"
            0x7E => 0x00,
            // Reserved: "no specific value is guaranteed when they are read"
            _ => self.rng.byte(),
        }
    }

    /// One byte of a FIFO_DATA burst (sections 3.6.3 and 3.6.6)
    fn fifo_data_byte(&mut self) -> u8 {
        self.burst.active = true;
        let stored = self.burst.overread.is_none() && !self.fifo.is_empty();
        let byte = if stored {
            let head = self.fifo.front().unwrap();
            let byte = head[self.head_read];
            self.head_read += 1;
            if self.head_read == head.len() {
                let frame = self.fifo.pop_front().unwrap();
                self.served.consumed.push(frame);
                self.head_read = 0;
                self.update_fifo_interrupts();
            }
            byte
        } else {
            if self.burst.overread.is_none() {
                // After the last frame: one sensor time frame if fifo_time_en, then
                // empty frames for as long as the burst lasts (section 3.6.6.3)
                let frame = if self.fifo_config_1 & 0b100 != 0 && !self.burst.time_sent {
                    self.burst.time_sent = true;
                    let mut frame = vec![0xA0];
                    frame.extend_from_slice(&self.sensor_time().to_le_bytes()[..3]);
                    frame
                } else {
                    vec![0x80, 0x00]
                };
                self.burst.overread = Some((frame, 0));
            }
            let (frame, pos) = self.burst.overread.as_mut().unwrap();
            let byte = frame[*pos];
            *pos += 1;
            if *pos == frame.len() {
                self.burst.overread = None;
            }
            byte
        };
        self.burst.bytes.push(byte);
        byte
    }

    fn write_register(&mut self, addr: u8, value: u8) {
        self.served.writes.push((addr, value));
        let reserved = |mask: u8, name: &str| {
            assert_eq!(
                value & mask,
                0,
                "reserved bits of {name} written: 0x{value:02x}"
            );
        };
        match addr {
            0x15 => {
                self.fifo_wtm = self.fifo_wtm & 0x100 | u16::from(value);
                self.update_fifo_interrupts();
            }
            0x16 => {
                reserved(0b1111_1110, "FIFO_WTM_1");
                self.fifo_wtm = self.fifo_wtm & 0xFF | u16::from(value & 1) << 8;
                self.update_fifo_interrupts();
            }
            0x17 => {
                reserved(0b1110_0000, "FIFO_CONFIG_1");
                self.fifo_config_1 = value;
            }
            0x18 => {
                reserved(0b1110_0000, "FIFO_CONFIG_2");
                self.track_change(self.fifo_config_2, value);
                self.fifo_config_2 = value;
            }
            0x19 => {
                reserved(0b1000_0000, "INT_CTRL");
                self.int_ctrl = value;
            }
            0x1A => {
                reserved(0b1111_1000, "IF_CONF");
                assert!(
                    !(self.spi && value & 1 != 0),
                    "spi3 set on a 4-wire SPI bus (IF_CONF)"
                );
                self.if_conf = value;
            }
            0x1B => {
                reserved(0b1100_1100, "PWR_CTRL");
                self.track_change(self.enables, value & 0b11);
                self.enables = value & 0b11;
                self.served.sleep_written |= Mode::from_bits(value >> 4) == Mode::Sleep;
                self.mode_command(Mode::from_bits(value >> 4));
            }
            0x1C => {
                reserved(0b1100_0000, "OSR");
                assert!(
                    value & 0b111 <= 0b101 && (value >> 3) & 0b111 <= 0b101,
                    "undefined oversampling in OSR: 0x{value:02x} (table 43)"
                );
                self.track_change(self.osr, value);
                self.osr = value;
                self.osr_written_us = self.now_us;
            }
            0x1D => {
                reserved(0b1110_0000, "ODR");
                self.track_change(self.odr, value);
                self.odr = value;
            }
            0x1F => {
                reserved(0b1111_0001, "CONFIG");
                self.track_change(self.config, value);
                self.config = value;
            }
            0x7E => match value {
                0x00 => {}
                0xB0 => {
                    // fifo_flush: "Clears all data in the FIFO"
                    self.fifo.clear();
                    self.head_read = 0;
                }
                0xB6 => {
                    self.served.soft_resets += 1;
                    self.power_on_reset();
                    self.busy_until_us = self.now_us + SOFT_RESET_US;
                }
                _ => self.cmd_err = true,
            },
            _ => panic!("write of 0x{value:02x} to read-only or reserved register 0x{addr:02x}"),
        }
    }

    /// "Configuration changes are tracked when the device is in normal mode" (section 3.6.5)
    fn track_change(&mut self, old: u8, new: u8) {
        if old != new && self.mode == Mode::Normal && self.fifo_enabled() {
            self.fifo_config_changed = true;
        }
    }

    /// Register writes come in pairs of register address and data (sections 5.2.1 and 5.3.1)
    fn write_pairs(&mut self, bytes: &[u8]) {
        assert!(
            bytes.len().is_multiple_of(2),
            "write of {} bytes is not a sequence of address/data pairs: {bytes:02x?}",
            bytes.len()
        );
        for pair in bytes.chunks(2) {
            self.write_register(pair[0] & 0x7F, pair[1]);
        }
    }

    fn delay_ns(&mut self, ns: u32) {
        self.advance_to(self.now_us + u64::from(ns).div_ceil(1000));
    }
}

// --- Bus adapters ---

struct SpiBus<'a>(&'a RefCell<Model>);
struct I2cBus<'a>(&'a RefCell<Model>);
struct Clock<'a>(&'a RefCell<Model>);

impl embedded_hal_async::spi::ErrorType for SpiBus<'_> {
    type Error = embedded_hal_async::spi::ErrorKind;
}

/// SPI: the first byte has the R/W bit and the 7-bit address. A read sends one
/// dummy byte before the data (section 5.3.2); a write is a sequence of
/// control byte/data pairs (section 5.3.1).
impl embedded_hal_async::spi::SpiDevice for SpiBus<'_> {
    async fn transaction(
        &mut self,
        operations: &mut [embedded_hal_async::spi::Operation<'_, u8>],
    ) -> Result<(), Self::Error> {
        use embedded_hal_async::spi::Operation;
        let mut model = self.0.borrow_mut();
        model
            .begin()
            .map_err(|()| embedded_hal_async::spi::ErrorKind::Other)?;
        let mut written = Vec::new();
        let mut read_addr = None;
        for op in operations.iter_mut() {
            match op {
                Operation::Write(bytes) => {
                    assert!(read_addr.is_none(), "write after the read phase");
                    written.extend_from_slice(bytes);
                }
                Operation::Read(buf) => {
                    assert!(
                        read_addr.is_none(),
                        "two read operations in one transaction"
                    );
                    assert_eq!(written.len(), 1, "read after {written:02x?}");
                    assert!(written[0] & 0x80 != 0, "read without the R/W bit");
                    let addr = written[0] & 0x7F;
                    read_addr = Some(addr);
                    assert!(!buf.is_empty());
                    // The dummy byte carries no data
                    buf[0] = model.rng.byte();
                    model.read_burst(addr, &mut buf[1..]);
                }
                other => panic!("unexpected SPI operation {other:?}"),
            }
        }
        if read_addr.is_none() {
            assert!(
                written.iter().step_by(2).all(|control| control & 0x80 == 0),
                "write with the R/W bit set: {written:02x?}"
            );
            model.write_pairs(&written);
        }
        model.end();
        Ok(())
    }
}

impl embedded_hal_async::i2c::ErrorType for I2cBus<'_> {
    type Error = embedded_hal_async::i2c::ErrorKind;
}

/// I2C: a one-byte write sets the register address of the next read; a longer
/// write is a sequence of address/data pairs (sections 5.2.1 and 5.2.2)
impl embedded_hal_async::i2c::I2c for I2cBus<'_> {
    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal_async::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        use embedded_hal_async::i2c::{ErrorKind, NoAcknowledgeSource, Operation};
        let mut model = self.0.borrow_mut();
        assert_eq!(model.i2c_address, address, "wrong I2C address");
        model
            .begin()
            .map_err(|()| ErrorKind::NoAcknowledge(NoAcknowledgeSource::Data))?;
        for op in operations.iter_mut() {
            match op {
                Operation::Write(bytes) if bytes.len() == 1 => model.i2c_pointer = bytes[0],
                Operation::Write(bytes) => model.write_pairs(bytes),
                Operation::Read(buf) => {
                    let addr = model.i2c_pointer;
                    model.read_burst(addr, buf);
                }
            }
        }
        model.end();
        Ok(())
    }
}

impl DelayNs for Clock<'_> {
    async fn delay_ns(&mut self, ns: u32) {
        self.0.borrow_mut().delay_ns(ns);
    }
}

/// The bus and the clock never return `Pending`, so one poll completes any
/// driver future
fn ready<F: Future>(future: F) -> F::Output {
    match pin!(future).poll(&mut Context::from_waker(Waker::noop())) {
        Poll::Ready(output) => output,
        Poll::Pending => unreachable!("the mocks never return Pending"),
    }
}

// --- Datasheet decoders ---

/// Compensation of the appendix (sections 8.4 to 8.6) in f64; returns the value
/// and the sum of the magnitudes of its terms, for the rounding tolerance of f32
struct Compensation {
    t1: f64,
    t2: f64,
    t3: f64,
    p: [f64; 11],
}

impl Compensation {
    fn new(nvm: &[u8; 21]) -> Self {
        let u16_at = |i: usize| f64::from(u16::from_le_bytes([nvm[i], nvm[i + 1]]));
        let i16_at = |i: usize| f64::from(i16::from_le_bytes([nvm[i], nvm[i + 1]]));
        let i8_at = |i: usize| f64::from(nvm[i] as i8);
        let two = |e: i32| 2f64.powi(e);
        // Table 24: T1 0x31, T2 0x33, T3 0x35, P1 0x36, P2 0x38, P3 0x3A, P4 0x3B,
        // P5 0x3C, P6 0x3E, P7 0x40, P8 0x41, P9 0x42, P10 0x44, P11 0x45
        Self {
            t1: u16_at(0) / two(-8),
            t2: u16_at(2) / two(30),
            t3: i8_at(4) / two(48),
            p: [
                (i16_at(5) - two(14)) / two(20),
                (i16_at(7) - two(14)) / two(29),
                i8_at(9) / two(32),
                i8_at(10) / two(37),
                u16_at(11) / two(-3),
                u16_at(13) / two(6),
                i8_at(15) / two(8),
                i8_at(16) / two(15),
                i16_at(17) / two(48),
                i8_at(19) / two(48),
                i8_at(20) / two(65),
            ],
        }
    }

    fn temperature(&self, raw: u32) -> (f64, f64) {
        let d1 = f64::from(raw) - self.t1;
        let a = d1 * self.t2;
        let b = d1 * d1 * self.t3;
        (a + b, a.abs() + b.abs())
    }

    fn pressure(&self, raw: u32, t_lin: f64) -> (f64, f64) {
        let [p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11] = self.p;
        let r = f64::from(raw);
        let terms = [
            p5,
            p6 * t_lin,
            p7 * t_lin * t_lin,
            p8 * t_lin * t_lin * t_lin,
            r * p1,
            r * p2 * t_lin,
            r * p3 * t_lin * t_lin,
            r * p4 * t_lin * t_lin * t_lin,
            r * r * p9,
            r * r * p10 * t_lin,
            r * r * r * p11,
        ];
        (terms.iter().sum(), terms.iter().map(|t| t.abs()).sum())
    }
}

fn assert_close(what: &str, expected: (f64, f64), actual: f32) {
    let (value, magnitude) = expected;
    let tolerance = magnitude * 2e-6 + 1e-3;
    assert!(
        (value - f64::from(actual)).abs() <= tolerance,
        "{what}: expected {value}, the driver returned {actual}"
    );
}

fn u24(bytes: &[u8]) -> u32 {
    u32::from(bytes[0]) | u32::from(bytes[1]) << 8 | u32::from(bytes[2]) << 16
}

// --- Driver calls ---

/// A configuration in register terms
#[derive(Clone, Copy, Debug)]
struct Settings {
    press_en: bool,
    temp_en: bool,
    mode: Mode,
    odr: u8,
    osr_p: u8,
    osr_t: u8,
    iir: u8,
}

impl Settings {
    fn random(rng: &mut Rng) -> Self {
        let press_en = !rng.one_in(6);
        let temp_en = !rng.one_in(6);
        let osr_p = rng.below(6) as u8;
        let osr_t = rng.below(6) as u8;
        let mode = match rng.below(4) {
            0 => Mode::Sleep,
            1 => Mode::Forced,
            _ => Mode::Normal,
        };
        // Mostly a rate that fits the measurement time, often the fastest ones
        let fastest = (0..=17u8)
            .find(|&odr| fits_normal_mode(press_en, temp_en, osr_p, osr_t, odr))
            .unwrap();
        let odr = match rng.below(8) {
            0 => rng.below(18) as u8,
            1..=4 => (fastest + rng.below(3) as u8).min(17),
            _ => fastest + rng.below(u64::from(18 - fastest)) as u8,
        };
        Self {
            press_en,
            temp_en,
            mode,
            odr,
            osr_p,
            osr_t,
            iir: rng.below(8) as u8,
        }
    }

    fn fits_normal_mode(&self) -> bool {
        fits_normal_mode(
            self.press_en,
            self.temp_en,
            self.osr_p,
            self.osr_t,
            self.odr,
        )
    }

    fn configuration(&self) -> Configuration {
        Configuration::default()
            .enable_pressure_measurement(self.press_en)
            .enable_temperature_measurement(self.temp_en)
            .power_mode(power_mode(self.mode))
            .output_data_rate(ODR[usize::from(self.odr)])
            .pressure_oversampling(OSR[usize::from(self.osr_p)])
            .temperature_oversampling(OSR[usize::from(self.osr_t)])
            .iir_filter_coefficient(IIR[usize::from(self.iir)])
    }

    /// The register values that the datasheet expects for this configuration
    fn check_registers(&self, model: &Model) {
        assert_eq!(
            u8::from(self.press_en) | u8::from(self.temp_en) << 1,
            model.enables,
            "press_en/temp_en"
        );
        assert_eq!(self.osr_p | self.osr_t << 3, model.osr, "OSR");
        assert_eq!(self.odr, model.odr, "ODR");
        assert_eq!(self.iir << 1, model.config, "CONFIG");
    }
}

fn power_mode(mode: Mode) -> PowerMode {
    match mode {
        Mode::Sleep => PowerMode::Sleep,
        Mode::Forced => PowerMode::Forced,
        Mode::Normal => PowerMode::Normal,
    }
}

/// The driver enums indexed by their register value; tables 43, 45 and 46
const OSR: [Oversampling; 6] = [
    Oversampling::X1,
    Oversampling::X2,
    Oversampling::X4,
    Oversampling::X8,
    Oversampling::X16,
    Oversampling::X32,
];

const ODR: [OutputDataRate; 18] = [
    OutputDataRate::R200Hz,
    OutputDataRate::R100Hz,
    OutputDataRate::R50Hz,
    OutputDataRate::R25Hz,
    OutputDataRate::R12p5Hz,
    OutputDataRate::R6p25Hz,
    OutputDataRate::R3p1Hz,
    OutputDataRate::R1p5Hz,
    OutputDataRate::R0p78Hz,
    OutputDataRate::R0p39Hz,
    OutputDataRate::R0p2Hz,
    OutputDataRate::R0p1Hz,
    OutputDataRate::R0p05Hz,
    OutputDataRate::R0p02Hz,
    OutputDataRate::R0p01Hz,
    OutputDataRate::R0p006Hz,
    OutputDataRate::R0p003Hz,
    OutputDataRate::R0p0015Hz,
];

const IIR: [IIRFilterCoefficient; 8] = [
    IIRFilterCoefficient::Coef0,
    IIRFilterCoefficient::Coef1,
    IIRFilterCoefficient::Coef3,
    IIRFilterCoefficient::Coef7,
    IIRFilterCoefficient::Coef15,
    IIRFilterCoefficient::Coef31,
    IIRFilterCoefficient::Coef63,
    IIRFilterCoefficient::Coef127,
];

#[derive(Clone, Copy, Debug)]
enum Op {
    New(Settings, bool),
    ApplyConfiguration(Settings),
    SetMode(Mode),
    GetMode,
    SetOversampling(u8, u8),
    GetOversampling,
    SetOutputDataRate(u8),
    GetOutputDataRate,
    SetIir(u8),
    GetIir,
    SetPressureEnabled(bool),
    GetPressureEnabled,
    SetTemperatureEnabled(bool),
    GetTemperatureEnabled,
    ReadSensorData,
    InterruptStatus,
    Status,
    ErrorFlags,
    IsConnected,
    CommandReady,
    SoftReset,
    SetFifoConfiguration {
        mode: bool,
        press: bool,
        temp: bool,
        time: bool,
        stop: bool,
        subsampling: u8,
        filtered: bool,
    },
    GetFifoConfiguration,
    FifoLength,
    ReadFifoFrame,
    /// `ReadFifoFrame` until the FIFO is empty; each read is a step of its own
    DrainFifo,
    /// `SetOversampling` to a short measurement and at once `ApplyConfiguration`: the
    /// running measurement keeps the old, longer oversampling
    LowerThenApply(u8, u8, Settings),
    MaskInterrupts(u8),
    SetFifoWatermark(u16),
    GetFifoWatermark,
    FifoFlush,
    MaxMeasurementTime,
}

impl Op {
    fn random(rng: &mut Rng) -> Self {
        let ops: &[fn(&mut Rng) -> Op] = &[
            |r| Op::New(Settings::random(r), r.one_in(2)),
            |r| Op::ApplyConfiguration(Settings::random(r)),
            |r| {
                Op::SetMode(match r.below(3) {
                    0 => Mode::Sleep,
                    1 => Mode::Forced,
                    _ => Mode::Normal,
                })
            },
            |_| Op::GetMode,
            |r| Op::SetOversampling(r.below(6) as u8, r.below(6) as u8),
            |_| Op::GetOversampling,
            |r| Op::SetOutputDataRate(r.below(18) as u8),
            |_| Op::GetOutputDataRate,
            |r| Op::SetIir(r.below(8) as u8),
            |_| Op::GetIir,
            |r| Op::SetPressureEnabled(!r.one_in(4)),
            |_| Op::GetPressureEnabled,
            |r| Op::SetTemperatureEnabled(!r.one_in(4)),
            |_| Op::GetTemperatureEnabled,
            |_| Op::ReadSensorData,
            |_| Op::ReadSensorData,
            |_| Op::InterruptStatus,
            |_| Op::Status,
            |_| Op::ErrorFlags,
            |_| Op::IsConnected,
            |_| Op::CommandReady,
            |_| Op::SoftReset,
            |r| Op::SetFifoConfiguration {
                mode: !r.one_in(4),
                press: r.one_in(2),
                temp: r.one_in(2),
                time: r.one_in(2),
                stop: r.one_in(2),
                subsampling: if r.one_in(4) { r.below(10) } else { r.below(2) } as u8,
                filtered: r.one_in(2),
            },
            |_| Op::GetFifoConfiguration,
            |_| Op::FifoLength,
            |_| Op::ReadFifoFrame,
            |_| Op::DrainFifo,
            |_| Op::DrainFifo,
            |r| Op::LowerThenApply(r.below(2) as u8, r.below(2) as u8, Settings::random(r)),
            |r| Op::LowerThenApply(r.below(2) as u8, r.below(2) as u8, Settings::random(r)),
            |r| Op::MaskInterrupts(r.below(8) as u8),
            |r| Op::SetFifoWatermark(r.below(512) as u16),
            |_| Op::GetFifoWatermark,
            |_| Op::FifoFlush,
            |_| Op::MaxMeasurementTime,
        ];
        ops[rng.below(ops.len() as u64) as usize](rng)
    }

    fn name(&self) -> &'static str {
        match self {
            Op::New(..) => "New",
            Op::ApplyConfiguration(_) => "ApplyConfiguration",
            Op::SetMode(_) => "SetMode",
            Op::GetMode => "GetMode",
            Op::SetOversampling(..) => "SetOversampling",
            Op::GetOversampling => "GetOversampling",
            Op::SetOutputDataRate(_) => "SetOutputDataRate",
            Op::GetOutputDataRate => "GetOutputDataRate",
            Op::SetIir(_) => "SetIir",
            Op::GetIir => "GetIir",
            Op::SetPressureEnabled(_) => "SetPressureEnabled",
            Op::GetPressureEnabled => "GetPressureEnabled",
            Op::SetTemperatureEnabled(_) => "SetTemperatureEnabled",
            Op::GetTemperatureEnabled => "GetTemperatureEnabled",
            Op::ReadSensorData => "ReadSensorData",
            Op::InterruptStatus => "InterruptStatus",
            Op::Status => "Status",
            Op::ErrorFlags => "ErrorFlags",
            Op::IsConnected => "IsConnected",
            Op::CommandReady => "CommandReady",
            Op::SoftReset => "SoftReset",
            Op::SetFifoConfiguration { .. } => "SetFifoConfiguration",
            Op::GetFifoConfiguration => "GetFifoConfiguration",
            Op::FifoLength => "FifoLength",
            Op::ReadFifoFrame | Op::DrainFifo => "ReadFifoFrame",
            Op::LowerThenApply(..) => "ApplyConfiguration",
            Op::MaskInterrupts(_) => "MaskInterrupts",
            Op::SetFifoWatermark(_) => "SetFifoWatermark",
            Op::GetFifoWatermark => "GetFifoWatermark",
            Op::FifoFlush => "FifoFlush",
            Op::MaxMeasurementTime => "MaxMeasurementTime",
        }
    }
}

/// Class of a driver error
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum Class {
    Bus,
    NotConnected,
    RegisterData,
    FifoData,
    Timeout,
    /// `TypeStateError::FifoConfigError`
    FifoConfig,
}

fn class<E>(error: &Bmp390Error<E>) -> Class {
    match error {
        Bmp390Error::Bus(_) => Class::Bus,
        Bmp390Error::NotConnected => Class::NotConnected,
        Bmp390Error::UnexpectedRegisterData(_) => Class::RegisterData,
        Bmp390Error::UnexpectedFifoData => Class::FifoData,
        Bmp390Error::Timeout => Class::Timeout,
    }
}

#[derive(Debug)]
enum Value {
    Unit,
    Bool(bool),
    Mode(PowerMode),
    Osr(OsrCfg),
    Odr(OutputDataRate),
    Iir(IIRFilterCoefficient),
    Data { pressure: f32, temperature: f32 },
    Flags(u8),
    Fifo(FifoConfiguration),
    Length(u16),
    Frame(FifoFrame),
    Time(u32),
}

/// What the test remembers about the driver between calls
struct Tracker {
    /// Raw temperature behind the cached t_lin of the driver; `None` = 0.0
    t_lin_raw: Option<u32>,
    /// The test configured the sensor only through the driver methods since the
    /// last soft reset, so the driver can know the measurement time
    known_config: bool,
}

/// Runs one driver call
fn call<B: Bus>(dev: &mut Bmp390<B>, op: Op, clock: &mut Clock) -> Result<Value, Class> {
    let r = |e: Bmp390Error<B::Error>| class(&e);
    Ok(match op {
        Op::New(..) | Op::DrainFifo | Op::LowerThenApply(..) => {
            unreachable!("handled by `run`")
        }
        Op::ApplyConfiguration(s) => {
            ready(dev.apply_configuration(&s.configuration(), clock)).map_err(r)?;
            Value::Unit
        }
        Op::SetMode(m) => {
            ready(dev.set_mode(power_mode(m))).map_err(r)?;
            Value::Unit
        }
        Op::GetMode => Value::Mode(ready(dev.mode()).map_err(r)?),
        Op::SetOversampling(p, t) => {
            let cfg = OsrCfg {
                osr_p: OSR[usize::from(p)],
                osr_t: OSR[usize::from(t)],
            };
            ready(dev.set_oversampling(&cfg)).map_err(r)?;
            Value::Unit
        }
        Op::GetOversampling => Value::Osr(ready(dev.oversampling()).map_err(r)?),
        Op::SetOutputDataRate(odr) => {
            ready(dev.set_output_data_rate(ODR[usize::from(odr)])).map_err(r)?;
            Value::Unit
        }
        Op::GetOutputDataRate => Value::Odr(ready(dev.output_data_rate()).map_err(r)?),
        Op::SetIir(c) => {
            ready(dev.set_iir_filter_coefficient(IIR[usize::from(c)])).map_err(r)?;
            Value::Unit
        }
        Op::GetIir => Value::Iir(ready(dev.iir_filter_coefficient()).map_err(r)?),
        Op::SetPressureEnabled(on) => {
            ready(dev.set_pressure_enabled(on)).map_err(r)?;
            Value::Unit
        }
        Op::GetPressureEnabled => Value::Bool(ready(dev.pressure_enabled()).map_err(r)?),
        Op::SetTemperatureEnabled(on) => {
            ready(dev.set_temperature_enabled(on)).map_err(r)?;
            Value::Unit
        }
        Op::GetTemperatureEnabled => Value::Bool(ready(dev.temperature_enabled()).map_err(r)?),
        Op::ReadSensorData => {
            let m = ready(dev.read_sensor_data()).map_err(r)?;
            Value::Data {
                pressure: m.pressure(),
                temperature: m.temperature(),
            }
        }
        Op::InterruptStatus => {
            let f = ready(dev.interrupt_status()).map_err(r)?;
            Value::Flags(u8::from(f.fwm_int) | u8::from(f.ffull_int) << 1 | u8::from(f.drdy) << 3)
        }
        Op::Status => {
            let f = ready(dev.status()).map_err(r)?;
            Value::Flags(
                u8::from(f.cmd_rdy) << 4 | u8::from(f.drdy_press) << 5 | u8::from(f.drdy_temp) << 6,
            )
        }
        Op::ErrorFlags => {
            let f = ready(dev.error_flags()).map_err(r)?;
            Value::Flags(
                u8::from(f.fatal_err) | u8::from(f.cmd_err) << 1 | u8::from(f.conf_err) << 2,
            )
        }
        Op::IsConnected => Value::Bool(ready(dev.is_connected()).map_err(r)?),
        Op::CommandReady => Value::Bool(ready(dev.command_ready()).map_err(r)?),
        Op::SoftReset => {
            ready(dev.soft_reset(clock)).map_err(r)?;
            Value::Unit
        }
        Op::SetFifoConfiguration {
            mode,
            press,
            temp,
            time,
            stop,
            subsampling,
            filtered,
        } => {
            let cfg = FifoConfiguration::default()
                .set_fifo_enabled(mode)
                .set_pressure_enabled(press)
                .set_temperature_enabled(temp)
                .set_time_enabled(time)
                .set_fifo_full_behavior(if stop {
                    FifoFullBehavior::Stop
                } else {
                    FifoFullBehavior::OverwriteOldest
                })
                .set_subsampling(subsampling)
                .set_apply_iir_filter(filtered);
            ready(dev.set_fifo_configuration(cfg)).map_err(r)?;
            Value::Unit
        }
        Op::GetFifoConfiguration => Value::Fifo(ready(dev.fifo_configuration()).map_err(r)?),
        Op::FifoLength => Value::Length(ready(dev.fifo_length()).map_err(r)?),
        Op::ReadFifoFrame => Value::Frame(ready(dev.read_fifo_frame()).map_err(r)?),
        Op::MaskInterrupts(bits) => {
            let mut ints = Interrupts::new();
            if bits & 1 != 0 {
                ints = ints.data_ready();
            }
            if bits & 2 != 0 {
                ints = ints.fifo_full();
            }
            if bits & 4 != 0 {
                ints = ints.fifo_watermark();
            }
            ready(dev.mask_interrupts(ints)).map_err(r)?;
            Value::Unit
        }
        Op::SetFifoWatermark(w) => {
            ready(dev.write::<FifoWtm>(&w)).map_err(r)?;
            Value::Unit
        }
        Op::GetFifoWatermark => Value::Length(ready(dev.read::<FifoWtm>()).map_err(r)?),
        Op::FifoFlush => {
            ready(dev.write::<Cmd>(&CmdData::FifoFlush)).map_err(r)?;
            Value::Unit
        }
        Op::MaxMeasurementTime => Value::Time(dev.max_measurement_time_us()),
    })
}

/// The sensor state before a call
struct Before {
    fits_normal_mode: bool,
    enables: u8,
    int_ctrl: u8,
    start_us: u64,
}

/// Compares the result of one driver call with what the model did
fn check(
    op: Op,
    before: &Before,
    result: &Result<Value, Class>,
    model: &Model,
    tracker: &mut Tracker,
) {
    let served = &model.served;

    // Every call: the driver does not hide a bus fault, and a configuration
    // error needs a configuration that does not fit normal mode
    if served.conf_errors > 0 {
        let requested_misfit = match op {
            Op::New(s, _) | Op::ApplyConfiguration(s) => !s.fits_normal_mode(),
            _ => false,
        };
        assert!(
            !before.fits_normal_mode || !model.fits_normal_mode() || requested_misfit,
            "configuration error in normal mode, but the old and the new configuration fit \
             (ERR_REG.conf_err); writes: {:02x?}",
            served.writes
        );
    }
    let forced_starts = served
        .started
        .iter()
        .filter(|(m, _)| *m == Mode::Forced)
        .count();
    let may_start_forced = matches!(op, Op::SetMode(Mode::Forced))
        || matches!(op, Op::New(s, _) | Op::ApplyConfiguration(s) if s.mode == Mode::Forced);
    assert!(
        forced_starts == 0 || may_start_forced,
        "{} started a forced measurement",
        op.name()
    );

    let value = match result {
        Err(Class::Bus) => {
            assert!(served.fault, "bus error without a bus fault");
            return;
        }
        Err(Class::NotConnected) => {
            assert!(
                matches!(op, Op::New(..)),
                "NotConnected outside of the probe"
            );
            return;
        }
        Err(Class::FifoData) => {
            assert!(
                matches!(op, Op::ReadFifoFrame),
                "UnexpectedFifoData outside of the FIFO"
            );
            return;
        }
        Err(e) => panic!("{} returned {e:?}", op.name()),
        Ok(value) => value,
    };
    let hidden = match op {
        Op::New(..) => served.fault_after_probe,
        _ => served.fault,
    };
    assert!(!hidden, "{} hid a bus fault", op.name());

    match (op, value) {
        (Op::New(s, _), _) | (Op::ApplyConfiguration(s), _) => {
            s.check_registers(model);
            match s.mode {
                Mode::Normal => {
                    assert_eq!(
                        model.mode,
                        Mode::Normal,
                        "not in normal mode; writes: {:02x?}",
                        served.writes
                    );
                    // Section 3.3.3, figure 3: after sleep -> normal the sensor
                    // measures at once. Without the transition the cycle keeps the
                    // phase of the old rate and the old settings.
                    if matches!(op, Op::New(..)) {
                        assert!(
                            model.normal_since_us.is_some_and(|t| t >= before.start_us),
                            "normal mode not entered from sleep: the measurement cycle did not \
                             restart; writes: {:02x?}",
                            served.writes
                        );
                    }
                }
                Mode::Sleep => assert_eq!(model.mode, Mode::Sleep, "not in sleep mode"),
                Mode::Forced => assert!(
                    forced_starts == 1,
                    "forced mode requested, {forced_starts} forced measurements; writes: {:02x?}",
                    served.writes
                ),
            }
            tracker.known_config = true;
        }
        (Op::SetMode(m), Value::Unit) => {
            let last = served
                .writes
                .iter()
                .rev()
                .find(|(a, _)| *a == 0x1B)
                .expect("no PWR_CTRL write");
            assert_eq!(m.bits(), last.1 >> 4, "mode bits");
            assert_eq!(
                before.enables,
                last.1 & 0b11,
                "set_mode changed press_en/temp_en"
            );
        }
        (Op::GetMode, Value::Mode(mode)) => {
            let raw = served.last_burst(0x1B)[0];
            assert_eq!(power_mode(Mode::from_bits(raw >> 4)), *mode);
        }
        (Op::SetOversampling(p, t), Value::Unit) => {
            assert_eq!(p | t << 3, model.osr, "OSR");
        }
        (Op::GetOversampling, Value::Osr(cfg)) => {
            let raw = served.last_burst(0x1C)[0];
            assert_eq!(OSR[usize::from(raw & 0b111)], cfg.osr_p);
            assert_eq!(OSR[usize::from(raw >> 3 & 0b111)], cfg.osr_t);
        }
        (Op::SetOutputDataRate(odr), Value::Unit) => assert_eq!(odr, model.odr, "ODR"),
        (Op::GetOutputDataRate, Value::Odr(odr)) => {
            let raw = served.last_burst(0x1D)[0];
            assert_eq!(ODR[usize::from(raw & 0x1F).min(17)], *odr);
        }
        (Op::SetIir(c), Value::Unit) => assert_eq!(c << 1, model.config, "CONFIG"),
        (Op::GetIir, Value::Iir(c)) => {
            let raw = served.last_burst(0x1F)[0];
            assert_eq!(IIR[usize::from(raw >> 1 & 0b111)], *c);
        }
        (Op::SetPressureEnabled(on), Value::Unit) => {
            assert_eq!(on, model.enables & 1 != 0);
            assert_eq!(before.enables & 2, model.enables & 2, "temp_en changed");
        }
        (Op::SetTemperatureEnabled(on), Value::Unit) => {
            assert_eq!(on, model.enables & 2 != 0);
            assert_eq!(before.enables & 1, model.enables & 1, "press_en changed");
        }
        (Op::GetPressureEnabled, Value::Bool(on)) => {
            assert_eq!(served.last_burst(0x1B)[0] & 1 != 0, *on);
        }
        (Op::GetTemperatureEnabled, Value::Bool(on)) => {
            assert_eq!(served.last_burst(0x1B)[0] & 2 != 0, *on);
        }
        (
            Op::ReadSensorData,
            Value::Data {
                pressure,
                temperature,
            },
        ) => {
            // Section 3.10: one burst from press_xlsb to temp_msb
            let raw = served.last_burst(0x04);
            assert_eq!(raw.len(), 6, "data read is not one burst of 6 bytes");
            let comp = Compensation::new(&model.nvm);
            let (t_lin, t_mag) = comp.temperature(u24(&raw[3..6]));
            assert_close("temperature", (t_lin, t_mag), *temperature);
            assert_close("pressure", comp.pressure(u24(&raw[0..3]), t_lin), *pressure);
            tracker.t_lin_raw = Some(u24(&raw[3..6]));
        }
        (Op::InterruptStatus, Value::Flags(f)) => {
            assert_eq!(served.last_burst(0x11)[0] & 0b1011, *f);
        }
        (Op::Status, Value::Flags(f)) => {
            assert_eq!(served.last_burst(0x03)[0] & 0b0111_0000, *f);
        }
        (Op::ErrorFlags, Value::Flags(f)) => {
            assert_eq!(served.last_burst(0x02)[0] & 0b111, *f);
        }
        (Op::IsConnected, Value::Bool(yes)) => {
            let id = served.last_burst(0x00)[0];
            assert_eq!(id == BMP390_CHIP_ID || id == BMP388_CHIP_ID, *yes);
        }
        (Op::CommandReady, Value::Bool(yes)) => {
            assert_eq!(served.last_burst(0x03)[0] & 0x10 != 0, *yes);
        }
        (Op::SoftReset, Value::Unit) => {
            assert_eq!(served.soft_resets, 1, "soft reset command");
            assert!(
                model.now_us >= model.busy_until_us,
                "soft_reset returned before the reset ended"
            );
        }
        (
            Op::SetFifoConfiguration {
                mode,
                press,
                temp,
                time,
                stop,
                subsampling,
                filtered,
            },
            Value::Unit,
        ) => {
            // Tables 38 and 39
            let c1 = u8::from(mode)
                | u8::from(stop) << 1
                | u8::from(time) << 2
                | u8::from(press) << 3
                | u8::from(temp) << 4;
            assert_eq!(c1, model.fifo_config_1, "FIFO_CONFIG_1");
            // `set_subsampling`: "This value is clamped to the range 0-7"
            let c2 = subsampling.min(7) | u8::from(filtered) << 3;
            assert_eq!(
                c2, model.fifo_config_2,
                "FIFO_CONFIG_2 for subsampling {subsampling}"
            );
        }
        (Op::GetFifoConfiguration, Value::Fifo(cfg)) => {
            let c1 = served.last_burst(0x17)[0];
            let c2 = served.last_burst(0x18)[0];
            assert_eq!(c1 & 1 != 0, cfg.fifo_enabled());
            assert_eq!(
                c1 & 2 != 0,
                cfg.fifo_full_behavior() == FifoFullBehavior::Stop
            );
            assert_eq!(c1 & 4 != 0, cfg.time_enabled());
            assert_eq!(c1 & 8 != 0, cfg.pressure_enabled());
            assert_eq!(c1 & 16 != 0, cfg.temperature_enabled());
            assert_eq!(c2 & 0b111, cfg.subsampling());
            assert_eq!(c2 >> 3 & 0b11 == 0b01, cfg.apply_iir_filter());
        }
        (Op::FifoLength, Value::Length(len)) => {
            let raw = served.last_burst(0x12);
            assert_eq!(u16::from(raw[0]) | u16::from(raw[1] & 1) << 8, *len);
        }
        (Op::ReadFifoFrame, Value::Frame(frame)) => check_fifo_frame(frame, model, tracker),
        (Op::MaskInterrupts(bits), Value::Unit) => {
            // Table 40: drdy_en bit 6, ffull_en bit 4, fwtm_en bit 3; other bits kept
            let expected = before.int_ctrl & !0b0101_1000
                | u8::from(bits & 1 == 0) << 6
                | u8::from(bits & 2 == 0) << 4
                | u8::from(bits & 4 == 0) << 3;
            assert_eq!(expected, model.int_ctrl, "INT_CTRL");
        }
        (Op::SetFifoWatermark(w), Value::Unit) => assert_eq!(w, model.fifo_wtm, "watermark"),
        (Op::GetFifoWatermark, Value::Length(w)) => {
            let raw = served.last_burst(0x15);
            assert_eq!(u16::from(raw[0]) | u16::from(raw[1] & 1) << 8, *w);
        }
        (Op::FifoFlush, Value::Unit) => assert!(model.fifo.is_empty(), "FIFO not flushed"),
        (Op::MaxMeasurementTime, Value::Time(us)) => {
            if tracker.known_config {
                let (p, t, osr_p, osr_t) = model.measurement_settings();
                let max = max_conversion_us(p, t, osr_p, osr_t);
                // The method documents the typical time times 1.2
                let documented = (conversion_us(p, t, osr_p, osr_t) * 12).div_ceil(10);
                assert!(
                    (max..=documented).contains(&u64::from(*us)),
                    "max_measurement_time_us() = {us}, but the measurement takes up to {max} us, \
                     documented {documented} us (OSR 0x{:02x}, press_en {p}, temp_en {t})",
                    model.osr
                );
            }
        }
        (op, value) => panic!("{} returned {value:?}", op.name()),
    }
}

/// The frame must be the first frame of the last FIFO_DATA burst, read whole,
/// and no other stored frame may be lost
fn check_fifo_frame(frame: &FifoFrame, model: &Model, tracker: &mut Tracker) {
    let served = &model.served;
    let burst = served.fifo_bursts.last().expect("no FIFO_DATA burst");
    let header = burst[0];
    let size = match header {
        0x44 | 0x48 | 0x80 => 2,
        0xA0 | 0x84 | 0x90 => 4,
        0x94 => 7,
        h => panic!("the model sent an unknown header 0x{h:02x}"),
    };
    assert!(
        burst.len() >= size,
        "partial frame decoded: header 0x{header:02x}, {} bytes read",
        burst.len()
    );
    // A frame that follows a short frame in the same burst is consumed too. That
    // happens only if the oldest frame changed after the driver read its header,
    // because the read size comes from that header.
    let race = served.fifo_bursts.windows(2).any(|w| w[0][0] != w[1][0]);
    let stored = &served.consumed;
    match header {
        0x80 | 0xA0 => assert!(
            stored.is_empty() || race,
            "stored frames lost: {stored:02x?}; bursts {:02x?}",
            served.fifo_bursts
        ),
        _ => assert!(
            stored.first() == Some(&burst[..size].to_vec()) && (stored.len() == 1 || race),
            "the driver consumed {stored:02x?}, but returned the first frame of bursts {:02x?}",
            served.fifo_bursts
        ),
    }
    let comp = Compensation::new(&model.nvm);
    let t_lin = |tracker: &Tracker| tracker.t_lin_raw.map_or(0.0, |raw| comp.temperature(raw).0);
    match (header, frame) {
        (0x44, FifoFrame::ControlFrame(ControlFrameType::ConfigError)) => {}
        (0x48, FifoFrame::ControlFrame(ControlFrameType::ConfigChange)) => {}
        (0x80, FifoFrame::SensorFrame(SensorFrameType::Empty)) => {}
        (0xA0, FifoFrame::SensorFrame(SensorFrameType::SensorTime(t))) => {
            assert_eq!(u24(&burst[1..4]), *t, "sensor time");
        }
        (0x90, FifoFrame::SensorFrame(SensorFrameType::Temperature(t))) => {
            let raw = u24(&burst[1..4]);
            assert_close("FIFO temperature", comp.temperature(raw), *t);
            tracker.t_lin_raw = Some(raw);
        }
        (0x84, FifoFrame::SensorFrame(SensorFrameType::Pressure(p))) => {
            assert_close(
                "FIFO pressure",
                comp.pressure(u24(&burst[1..4]), t_lin(tracker)),
                *p,
            );
        }
        (
            0x94,
            FifoFrame::SensorFrame(SensorFrameType::PressureAndTemperature {
                pressure,
                temperature,
            }),
        ) => {
            let raw_t = u24(&burst[1..4]);
            let (tl, mag) = comp.temperature(raw_t);
            assert_close("FIFO temperature", (tl, mag), *temperature);
            assert_close(
                "FIFO pressure",
                comp.pressure(u24(&burst[4..7]), tl),
                *pressure,
            );
            tracker.t_lin_raw = Some(raw_t);
        }
        (h, f) => panic!(
            "header 0x{h:02x} decoded as {f:?}; bursts {:02x?}",
            served.fifo_bursts
        ),
    }
}

/// Proves that the random runs reach every call and every error path
#[derive(Default)]
struct Coverage {
    ok: std::collections::BTreeMap<&'static str, usize>,
    errors: std::collections::BTreeMap<Class, usize>,
    conf_errors: usize,
    frames: std::collections::BTreeSet<u8>,
    /// FIFO reads where the oldest frame changed between two bursts of one call
    fifo_races: usize,
    /// Calls that found the device still awake after the computed wait for sleep
    late_sleeps: usize,
}

impl Coverage {
    fn record(&mut self, op: Op, result: &Result<Value, Class>, model: &Model) {
        match result {
            Ok(_) => *self.ok.entry(op.name()).or_default() += 1,
            Err(c) => *self.errors.entry(*c).or_default() += 1,
        }
        self.conf_errors += model.served.conf_errors;
        if let (Op::ReadFifoFrame, Ok(_)) = (op, result) {
            self.frames
                .insert(model.served.fifo_bursts.last().unwrap()[0]);
        }
        if matches!(op, Op::ReadFifoFrame) {
            let bursts = &model.served.fifo_bursts;
            if bursts.windows(2).any(|w| w[0][0] != w[1][0]) {
                self.fifo_races += 1;
            }
        }
        if model.served.not_asleep_reads > 0 {
            self.late_sleeps += 1;
        }
    }

    fn assert_complete(&self) {
        if std::env::var("BMP390_SIM_SEED").is_ok() {
            return;
        }
        let mut rng = Rng::new(0);
        for _ in 0..10_000 {
            let name = Op::random(&mut rng).name();
            assert!(self.ok.contains_key(name), "{name} never succeeded");
        }
        for class in [Class::Bus, Class::NotConnected] {
            assert!(self.errors.contains_key(&class), "no {class:?}");
        }
        assert!(self.conf_errors > 0, "no configuration error");
        for header in [0x44, 0x48, 0x80, 0xA0, 0x84, 0x90, 0x94] {
            assert!(
                self.frames.contains(&header),
                "no FIFO frame 0x{header:02x}"
            );
        }
        eprintln!(
            "coverage: errors {:?}, conf_errors {}, fifo_races {}, late_sleeps {}",
            self.errors, self.conf_errors, self.fifo_races, self.late_sleeps
        );
        assert!(
            self.fifo_races > 0,
            "no FIFO frame changed between two bursts"
        );
        assert!(
            self.late_sleeps > 0,
            "the device was always asleep after the computed wait"
        );
    }
}

/// Prints the seed if the simulation panics
struct SeedReport(u64);

impl Drop for SeedReport {
    fn drop(&mut self) {
        if std::thread::panicking() {
            eprintln!("failing seed: BMP390_SIM_SEED={}", self.0);
        }
    }
}

fn seeds() -> core::ops::Range<u64> {
    match std::env::var("BMP390_SIM_SEED") {
        Ok(seed) => {
            let seed = seed.parse().expect("BMP390_SIM_SEED must be a number");
            seed..seed + 1
        }
        Err(_) => 0..SEEDS,
    }
}

/// Moves the clock between calls
fn pause(model: &RefCell<Model>, rng: &mut Rng) {
    let us = match rng.below(4) {
        0 => 0,
        1 => rng.below(100_000),
        2 => rng.below(2_000_000),
        _ => rng.below(40_000_000),
    };
    let mut model = model.borrow_mut();
    let t = model.now_us + us;
    model.advance_to(t);
}

fn before(model: &RefCell<Model>) -> Before {
    let mut model = model.borrow_mut();
    model.served = Served::default();
    Before {
        fits_normal_mode: model.fits_normal_mode(),
        enables: model.enables,
        int_ctrl: model.int_ctrl,
        start_us: model.now_us,
    }
}

/// One seeded run: `make` builds a driver on a fresh bus adapter
fn run<B: Bus>(
    seed: u64,
    model: &RefCell<Model>,
    coverage: &mut Coverage,
    make: impl Fn(Configuration, ResetPolicy) -> Result<Bmp390<B>, Class>,
) {
    let mut rng = Rng::new(!seed);
    let mut device: Option<Bmp390<B>> = None;
    let mut tracker = Tracker {
        t_lin_raw: None,
        known_config: false,
    };
    for _ in 0..STEPS {
        pause(model, &mut rng);
        let mut op = Op::random(&mut rng);
        if device.is_none() {
            op = Op::New(Settings::random(&mut rng), rng.one_in(2));
        }
        let steps = match op {
            Op::DrainFifo => vec![Op::ReadFifoFrame; FIFO_CAPACITY / 2],
            Op::LowerThenApply(p, t, s) => {
                vec![Op::SetOversampling(p, t), Op::ApplyConfiguration(s)]
            }
            op => vec![op],
        };
        let mut empty_reads = 0;
        for op in steps {
            let snapshot = before(model);
            let result = match op {
                Op::New(s, soft) => {
                    let reset = if soft {
                        ResetPolicy::Soft
                    } else {
                        ResetPolicy::None
                    };
                    match make(s.configuration(), reset) {
                        Ok(dev) => {
                            device = Some(dev);
                            tracker = Tracker {
                                t_lin_raw: None,
                                known_config: true,
                            };
                            Ok(Value::Unit)
                        }
                        Err(e) => Err(e),
                    }
                }
                _ => call(device.as_mut().unwrap(), op, &mut Clock(model)),
            };
            if let (Op::New(..), Err(Class::NotConnected)) = (op, &result) {
                let m = model.borrow();
                let real = m.chip_id == BMP390_CHIP_ID || m.chip_id == BMP388_CHIP_ID;
                assert!(
                    !real || m.served.fault,
                    "NotConnected for chip id 0x{:02x} without faults",
                    m.chip_id
                );
            }
            let m = model.borrow();
            if std::env::var("BMP390_SIM_TRACE").is_ok() {
                eprintln!(
                    "{:>12} us {op:?} -> {result:?}; mode {:?}, writes {:02x?}, started {:?}",
                    snapshot.start_us, m.mode, m.served.writes, m.served.started
                );
            }
            if let (Op::New(..), Err(_)) = (op, &result) {
                // The old driver does not know what the failed `new` wrote
                tracker.known_config = false;
            }
            check(op, &snapshot, &result, &m, &mut tracker);
            coverage.record(op, &result, &m);
            drop(m);
            if model.borrow().chip_id != BMP390_CHIP_ID && model.borrow().chip_id != BMP388_CHIP_ID
            {
                // No driver can be built for another chip; one probe is enough
                return;
            }
            // A polling consumer: stop after a few reads of an empty FIFO
            match result {
                Ok(Value::Frame(FifoFrame::SensorFrame(
                    SensorFrameType::Empty | SensorFrameType::SensorTime(_),
                ))) => empty_reads += 1,
                Err(_) => break,
                _ => {}
            }
            if empty_reads == 4 {
                break;
            }
        }
    }
}

#[test]
fn bmp390_spi_follows_the_datasheet() {
    let mut coverage = Coverage::default();
    for seed in seeds() {
        let _report = SeedReport(seed);
        let model = RefCell::new(Model::new(seed, true));
        run(seed, &model, &mut coverage, |config, reset| {
            ready(Bmp390::new_spi(
                SpiBus(&model),
                config,
                reset,
                &mut Clock(&model),
            ))
            .map_err(|e| class(&e))
        });
    }
    coverage.assert_complete();
}

#[test]
fn bmp390_i2c_follows_the_datasheet() {
    let mut coverage = Coverage::default();
    for seed in seeds() {
        let _report = SeedReport(seed);
        let model = RefCell::new(Model::new(seed ^ 0x5A5A, false));
        let sdo = if model.borrow().i2c_address == 0x77 {
            SdoPinState::High
        } else {
            SdoPinState::Low
        };
        run(seed, &model, &mut coverage, |config, reset| {
            ready(Bmp390::new_i2c(
                I2cBus(&model),
                sdo,
                config,
                reset,
                &mut Clock(&model),
            ))
            .map_err(|e| class(&e))
        });
    }
    coverage.assert_complete();
}

// --- Typestate interface ---

mod typestate {
    use super::*;
    use bmp390_rs::typestate::{Bmp390Builder, FifoOutput, TypeStateError};
    use core::convert::Infallible;

    const TS_STEPS: usize = 200;

    /// The INT pin of the model
    pub struct IntPin<'a>(pub &'a RefCell<Model>);

    impl embedded_hal::digital::ErrorType for IntPin<'_> {
        type Error = Infallible;
    }

    impl embedded_hal::digital::InputPin for IntPin<'_> {
        fn is_high(&mut self) -> Result<bool, Infallible> {
            let mut model = self.0.borrow_mut();
            model.gap();
            Ok(model.pin_high())
        }

        fn is_low(&mut self) -> Result<bool, Infallible> {
            self.is_high().map(|high| !high)
        }
    }

    impl embedded_hal_async::digital::Wait for IntPin<'_> {
        async fn wait_for_high(&mut self) -> Result<(), Infallible> {
            self.0.borrow_mut().wait_for_pin(true, false);
            Ok(())
        }

        async fn wait_for_low(&mut self) -> Result<(), Infallible> {
            self.0.borrow_mut().wait_for_pin(false, false);
            Ok(())
        }

        async fn wait_for_rising_edge(&mut self) -> Result<(), Infallible> {
            self.0.borrow_mut().wait_for_pin(true, true);
            Ok(())
        }

        async fn wait_for_falling_edge(&mut self) -> Result<(), Infallible> {
            self.0.borrow_mut().wait_for_pin(false, true);
            Ok(())
        }

        async fn wait_for_any_edge(&mut self) -> Result<(), Infallible> {
            unimplemented!("not used by the driver")
        }
    }

    #[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
    pub enum TsOp {
        ReadLatest,
        ReadNext,
        ReadMeasurement,
        Enqueue,
        Dequeue,
        Length,
        IsEmpty,
        SetOversampling(u8, u8),
        GetOversampling,
    }

    impl TsOp {
        fn kind(self) -> Self {
            match self {
                TsOp::SetOversampling(..) => TsOp::SetOversampling(0, 0),
                op => op,
            }
        }
    }

    #[derive(Debug)]
    pub enum TsValue {
        Unit,
        Data { pressure: f32, temperature: f32 },
        Time(u32),
        Empty,
        Length(u16),
        Bool(bool),
        Osr(OsrCfg),
    }

    pub fn ts_class<E, P>(error: TypeStateError<E, P>) -> Class {
        match error {
            TypeStateError::Device(e) => class(&e),
            TypeStateError::Pin(_) => unreachable!("the pin of the model does not fail"),
            TypeStateError::FifoConfigError => Class::FifoConfig,
        }
    }

    /// The sensor state before a call
    pub struct TsBefore {
        start_us: u64,
        measurements: u64,
        fits_normal_mode: bool,
        /// The running measurement started before the last OSR write, so it can
        /// take longer than the current settings say
        old_osr_running: bool,
    }

    pub fn ts_before(model: &RefCell<Model>) -> TsBefore {
        let mut model = model.borrow_mut();
        model.served = Served::default();
        TsBefore {
            start_us: model.now_us,
            measurements: model.measurements,
            fits_normal_mode: model.fits_normal_mode(),
            old_osr_running: model
                .running
                .is_some_and(|r| r.start_us < model.osr_written_us),
        }
    }

    /// The builder writes `Configuration::default()`, the FIFO configuration and the
    /// interrupt mask
    pub fn check_build(model: &Model, before: &TsBefore, normal: bool, fifo: bool, irq: bool) {
        // x8 pressure, x1 temperature, 50 Hz, IIR coefficient 15, both sensors
        assert_eq!(model.osr, 0b000_011, "OSR");
        assert_eq!(model.odr, 0x02, "ODR");
        assert_eq!(model.config, 0b100 << 1, "CONFIG");
        assert_eq!(model.enables, 0b11, "press_en/temp_en");
        if normal {
            assert_eq!(model.mode, Mode::Normal);
            assert!(
                model.normal_since_us.is_some_and(|t| t >= before.start_us),
                "cycle not restarted"
            );
        } else {
            assert_eq!(model.mode, Mode::Sleep);
        }
        if fifo {
            // fifo_mode, stop_on_full, time, pressure, temperature; subsampling 2, unfiltered
            assert_eq!(model.fifo_config_1, 0b1_1111, "FIFO_CONFIG_1");
            assert_eq!(model.fifo_config_2, 0x02, "FIFO_CONFIG_2");
        }
        if irq {
            assert_eq!(
                model.int_ctrl & 0b0101_1000,
                0b0100_0000,
                "INT_CTRL: only drdy_en"
            );
        }
    }

    pub fn check_ts(
        op: TsOp,
        before: &TsBefore,
        result: &Result<TsValue, Class>,
        model: &Model,
        last_ids: &mut Option<(u64, u64)>,
    ) {
        let served = &model.served;
        let value = match result {
            Err(Class::Bus) => {
                assert!(served.fault, "bus error without a bus fault");
                return;
            }
            Err(Class::FifoConfig) => {
                assert_eq!(op, TsOp::Dequeue);
                assert_eq!(
                    served.fifo_bursts.last().unwrap()[0],
                    0x44,
                    "FifoConfigError without the frame"
                );
                return;
            }
            Err(e) => panic!("{op:?} returned {e:?}"),
            Ok(value) => value,
        };
        assert!(!served.fault, "{op:?} hid a bus fault");
        let comp = Compensation::new(&model.nvm);
        let forced_starts = served
            .started
            .iter()
            .filter(|(m, _)| *m == Mode::Forced)
            .count();
        match (op, value) {
            (
                TsOp::ReadLatest | TsOp::ReadNext | TsOp::ReadMeasurement,
                TsValue::Data {
                    pressure,
                    temperature,
                },
            ) => {
                let raw = served.last_burst(0x04);
                let (t_lin, mag) = comp.temperature(u24(&raw[3..6]));
                assert_close("temperature", (t_lin, mag), *temperature);
                assert_close("pressure", comp.pressure(u24(&raw[0..3]), t_lin), *pressure);
                let ids = served.data_ids.unwrap();
                match op {
                    TsOp::ReadMeasurement => {
                        // "Triggers a new measurement and waits for the results"
                        assert_eq!(forced_starts, 1, "forced measurements started");
                        assert!(
                            ids.0 > before.measurements && ids.1 > before.measurements,
                            "read_measurement returned an older measurement: data of {ids:?}, \
                             {} measurements before the call",
                            before.measurements
                        );
                    }
                    TsOp::ReadNext
                        if before.fits_normal_mode
                            && model.fits_normal_mode()
                            && !before.old_osr_running =>
                    {
                        // "Reads the next measurement from the device"
                        if let Some(last) = *last_ids {
                            assert!(
                                ids.0 > last.0 && ids.1 > last.1,
                                "read_next_measurement returned data of {ids:?} again"
                            );
                        }
                    }
                    _ => {}
                }
                *last_ids = Some(ids);
            }
            (TsOp::Enqueue, TsValue::Unit) => {
                assert_eq!(forced_starts, 1, "forced measurements started")
            }
            (TsOp::Dequeue, value) => {
                let burst = served.fifo_bursts.last().expect("no FIFO_DATA burst");
                match (burst[0], value) {
                    (
                        0x94,
                        TsValue::Data {
                            pressure,
                            temperature,
                        },
                    ) => {
                        let (t_lin, mag) = comp.temperature(u24(&burst[1..4]));
                        assert_close("FIFO temperature", (t_lin, mag), *temperature);
                        let p = comp.pressure(u24(&burst[4..7]), t_lin);
                        assert_close("FIFO pressure", p, *pressure);
                    }
                    (0xA0, TsValue::Time(t)) => assert_eq!(u24(&burst[1..4]), *t),
                    (0x80, TsValue::Empty) => {}
                    (h, v) => panic!(
                        "header 0x{h:02x} dequeued as {v:?}; bursts {:02x?}",
                        served.fifo_bursts
                    ),
                }
            }
            (TsOp::Length, TsValue::Length(len)) => {
                let raw = served.last_burst(0x12);
                assert_eq!(u16::from(raw[0]) | u16::from(raw[1] & 1) << 8, *len);
            }
            (TsOp::IsEmpty, TsValue::Bool(empty)) => {
                let raw = served.last_burst(0x12);
                assert_eq!(raw[0] == 0 && raw[1] & 1 == 0, *empty);
            }
            (TsOp::SetOversampling(p, t), TsValue::Unit) => {
                assert_eq!(p | t << 3, model.osr, "OSR")
            }
            (TsOp::GetOversampling, TsValue::Osr(cfg)) => {
                let raw = served.last_burst(0x1C)[0];
                assert_eq!(OSR[usize::from(raw & 0b111)], cfg.osr_p);
                assert_eq!(OSR[usize::from(raw >> 3 & 0b111)], cfg.osr_t);
            }
            (op, value) => panic!("{op:?} returned {value:?}"),
        }
    }

    fn osr(p: u8, t: u8) -> OsrCfg {
        OsrCfg {
            osr_p: OSR[usize::from(p)],
            osr_t: OSR[usize::from(t)],
        }
    }

    /// One test per combination of mode, FIFO and interrupt pin: the typestate
    /// types differ, so does the set of methods
    macro_rules! variant {
        (
            $test:ident, normal: $normal:expr, fifo: $fifo:expr, irq: $irq:expr,
            ops: [$($ops:expr),* $(,)?],
            build: |$model:ident, $reset:ident| $build:expr,
            call: |$dev:ident| { $($op:pat => $body:expr),* $(,)? }
        ) => {
            #[test]
            fn $test() {
                let ops: &[TsOp] = &[$($ops),*];
                let mut coverage = std::collections::BTreeSet::new();
                for seed in seeds() {
                    let _report = SeedReport(seed);
                    let model = RefCell::new(Model::new(seed ^ 0x7E57_0000, true));
                    if !matches!(model.borrow().chip_id, BMP390_CHIP_ID | BMP388_CHIP_ID) {
                        continue;
                    }
                    let $model = &model;
                    let mut rng = Rng::new(!seed);
                    let mut $dev = loop {
                        let $reset = if rng.one_in(2) { ResetPolicy::Soft } else { ResetPolicy::None };
                        let snapshot = ts_before($model);
                        match ready($build) {
                            Ok(dev) => {
                                let m = model.borrow();
                                assert!(!m.served.fault_after_probe, "build hid a bus fault");
                                check_build(&m, &snapshot, $normal, $fifo, $irq);
                                break dev;
                            }
                            Err(e) => assert!(
                                model.borrow().served.fault,
                                "build failed: {:?}",
                                class(&e)
                            ),
                        }
                    };
                    let mut last_ids = None;
                    for _ in 0..TS_STEPS {
                        pause($model, &mut rng);
                        let op = match ops[rng.below(ops.len() as u64) as usize] {
                            TsOp::SetOversampling(..) => {
                                TsOp::SetOversampling(rng.below(6) as u8, rng.below(6) as u8)
                            }
                            op => op,
                        };
                        let snapshot = ts_before($model);
                        let result: Result<TsValue, Class> = match op {
                            $($op => $body,)*
                            op => unreachable!("{op:?} is not a call of this variant"),
                        };
                        if std::env::var("BMP390_SIM_TRACE").is_ok() {
                            let m = model.borrow();
                            eprintln!(
                                "{:>12} us {op:?} -> {result:?}; mode {:?}, writes {:02x?}",
                                snapshot.start_us, m.mode, m.served.writes
                            );
                        }
                        check_ts(op, &snapshot, &result, &model.borrow(), &mut last_ids);
                        if result.is_ok() {
                            coverage.insert(op.kind());
                        }
                    }
                }
                if std::env::var("BMP390_SIM_SEED").is_err() {
                    for op in ops {
                        assert!(coverage.contains(&op.kind()), "{op:?} never succeeded");
                    }
                }
            }
        };
    }

    /// The calls that every variant has
    macro_rules! data {
        ($result:expr, $map_err:expr) => {
            $result
                .map(|m| TsValue::Data {
                    pressure: m.pressure_pascal(),
                    temperature: m.temperature_celsius(),
                })
                .map_err($map_err)
        };
    }

    macro_rules! dequeued {
        ($result:expr) => {
            $result
                .map(|output| match output {
                    FifoOutput::SensorTime(t) => TsValue::Time(t),
                    FifoOutput::Empty => TsValue::Empty,
                    FifoOutput::Measurement(m) => TsValue::Data {
                        pressure: m.pressure_pascal(),
                        temperature: m.temperature_celsius(),
                    },
                })
                .map_err(ts_class)
        };
    }

    variant!(
        typestate_normal_follows_the_datasheet, normal: true, fifo: false, irq: false,
        ops: [TsOp::ReadLatest, TsOp::ReadNext, TsOp::ReadNext, TsOp::SetOversampling(0, 0), TsOp::GetOversampling],
        build: |model, reset| Bmp390Builder::new()
            .use_spi(SpiBus(model)).into_normal().enable_pressure().enable_temperature()
            .build(reset, Clock(model)),
        call: |dev| {
            TsOp::ReadLatest => data!(ready(dev.read_latest_measurement()), |e| class(&e)),
            TsOp::ReadNext => data!(ready(dev.read_next_measurement()), ts_class),
            TsOp::SetOversampling(p, t) => ready(dev.set_oversampling_config(&osr(p, t)))
                .map(|()| TsValue::Unit).map_err(|e| class(&e)),
            TsOp::GetOversampling => ready(dev.oversampling_config()).map(TsValue::Osr).map_err(|e| class(&e)),
        }
    );

    variant!(
        typestate_normal_irq_follows_the_datasheet, normal: true, fifo: false, irq: true,
        ops: [TsOp::ReadLatest, TsOp::ReadNext, TsOp::ReadNext, TsOp::SetOversampling(0, 0), TsOp::GetOversampling],
        build: |model, reset| Bmp390Builder::new()
            .use_spi(SpiBus(model)).into_normal().enable_pressure().enable_temperature()
            .use_irq(IntPin(model)).build(reset, Clock(model)),
        call: |dev| {
            TsOp::ReadLatest => data!(ready(dev.read_latest_measurement()), |e| class(&e)),
            TsOp::ReadNext => data!(ready(dev.read_next_measurement()), ts_class),
            TsOp::SetOversampling(p, t) => ready(dev.set_oversampling_config(&osr(p, t)))
                .map(|()| TsValue::Unit).map_err(|e| class(&e)),
            TsOp::GetOversampling => ready(dev.oversampling_config()).map(TsValue::Osr).map_err(|e| class(&e)),
        }
    );

    variant!(
        typestate_forced_follows_the_datasheet, normal: false, fifo: false, irq: false,
        ops: [TsOp::ReadMeasurement, TsOp::ReadMeasurement, TsOp::SetOversampling(0, 0), TsOp::GetOversampling],
        build: |model, reset| Bmp390Builder::new()
            .use_spi(SpiBus(model)).into_forced().enable_pressure().enable_temperature()
            .build(reset, Clock(model)),
        call: |dev| {
            TsOp::ReadMeasurement => data!(ready(dev.read_measurement()), ts_class),
            TsOp::SetOversampling(p, t) => ready(dev.set_oversampling_config(&osr(p, t)))
                .map(|()| TsValue::Unit).map_err(|e| class(&e)),
            TsOp::GetOversampling => ready(dev.oversampling_config()).map(TsValue::Osr).map_err(|e| class(&e)),
        }
    );

    variant!(
        typestate_forced_irq_follows_the_datasheet, normal: false, fifo: false, irq: true,
        ops: [TsOp::ReadMeasurement, TsOp::ReadMeasurement, TsOp::SetOversampling(0, 0), TsOp::GetOversampling],
        build: |model, reset| Bmp390Builder::new()
            .use_spi(SpiBus(model)).into_forced().enable_pressure().enable_temperature()
            .use_irq(IntPin(model)).build(reset, Clock(model)),
        call: |dev| {
            TsOp::ReadMeasurement => data!(ready(dev.read_measurement()), ts_class),
            TsOp::SetOversampling(p, t) => ready(dev.set_oversampling_config(&osr(p, t)))
                .map(|()| TsValue::Unit).map_err(|e| class(&e)),
            TsOp::GetOversampling => ready(dev.oversampling_config()).map(TsValue::Osr).map_err(|e| class(&e)),
        }
    );

    variant!(
        typestate_normal_fifo_follows_the_datasheet, normal: true, fifo: true, irq: false,
        ops: [TsOp::Dequeue, TsOp::Dequeue, TsOp::Length, TsOp::IsEmpty, TsOp::SetOversampling(0, 0), TsOp::GetOversampling],
        build: |model, reset| Bmp390Builder::new()
            .use_spi(SpiBus(model)).into_normal().enable_pressure().enable_temperature()
            .use_fifo().build(reset, Clock(model)),
        call: |dev| {
            TsOp::Dequeue => dequeued!(ready(dev.dequeue())),
            TsOp::Length => ready(dev.length()).map(TsValue::Length).map_err(ts_class),
            TsOp::IsEmpty => ready(dev.is_empty()).map(TsValue::Bool).map_err(ts_class),
            TsOp::SetOversampling(p, t) => ready(dev.set_oversampling_config(&osr(p, t)))
                .map(|()| TsValue::Unit).map_err(|e| class(&e)),
            TsOp::GetOversampling => ready(dev.oversampling_config()).map(TsValue::Osr).map_err(|e| class(&e)),
        }
    );

    variant!(
        typestate_forced_fifo_follows_the_datasheet, normal: false, fifo: true, irq: false,
        ops: [TsOp::Enqueue, TsOp::Dequeue, TsOp::Dequeue, TsOp::Length, TsOp::IsEmpty, TsOp::SetOversampling(0, 0), TsOp::GetOversampling],
        build: |model, reset| Bmp390Builder::new()
            .use_spi(SpiBus(model)).into_forced().enable_pressure().enable_temperature()
            .use_fifo().build(reset, Clock(model)),
        call: |dev| {
            TsOp::Enqueue => ready(dev.enqueue_measurement()).map(|()| TsValue::Unit).map_err(ts_class),
            TsOp::Dequeue => dequeued!(ready(dev.dequeue())),
            TsOp::Length => ready(dev.length()).map(TsValue::Length).map_err(ts_class),
            TsOp::IsEmpty => ready(dev.is_empty()).map(TsValue::Bool).map_err(ts_class),
            TsOp::SetOversampling(p, t) => ready(dev.set_oversampling_config(&osr(p, t)))
                .map(|()| TsValue::Unit).map_err(|e| class(&e)),
            TsOp::GetOversampling => ready(dev.oversampling_config()).map(TsValue::Osr).map_err(|e| class(&e)),
        }
    );
}
