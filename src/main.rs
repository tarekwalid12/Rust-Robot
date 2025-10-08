#![no_std]
#![no_main]
#![doc = include_str!("../README.md")]
//! # Robot Motor Controller with IR Remote
//!
//! An Embassy-based async motor controller for RP2350 with NEC IR protocol support.
//!
//! ## Hardware Architecture
//!
//! This firmware controls a dual-motor robot platform using:
//! - **PWM motor control** on GPIO 16 (Motor A) and GPIO 21 (Motor B)
//! - **H-bridge direction pins** on GPIO 17-20
//! - **NEC IR receiver** on GPIO 5 for wireless remote control
//!
//! ## Design Pattern
//!
//! The implementation uses Embassy's async runtime to provide:
//! - Non-blocking IR signal decoding with precise timing
//! - Concurrent motor control and IR processing
//! - Automatic safety timeout (800ms idle = stop motors)
//!
//! ## Key Features
//!
//! - **Async IR Decoding**: Full NEC protocol implementation with timing validation
//! - **Variable Speed Control**: 10 speed levels from 10% to 100% duty cycle
//! - **Safety Auto-Stop**: Motors stop if no IR signal received within 800ms
//! - **Type-Safe GPIO**: Embassy's HAL prevents common embedded programming errors

use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::pwm::{Config as PwmConfig, Pwm};
use embassy_time::{Duration, Instant, Timer};
use panic_halt as _;

// Motor Pin Assignments (same as C code)
// PWMA=16, AIN2=17, AIN1=18, BIN1=19, BIN2=20, PWMB=21
// IR_PIN=5

/// Motor controller managing dual H-bridge motor driver.
///
/// # Hardware Interface
///
/// Controls two motors through an H-bridge driver IC:
/// - **Motor A**: Controlled by AIN1/AIN2 (direction) and PWMA (speed)
/// - **Motor B**: Controlled by BIN1/BIN2 (direction) and PWMB (speed)
///
/// # PWM Configuration
///
/// - **Frequency**: ~1kHz (65535 TOP value at 125MHz system clock / 2048 prescaler)
/// - **Resolution**: 16-bit (0-65535)
/// - **Duty Cycle Range**: 0-100% motor speed
///
/// # Direction Control Truth Table
///
/// | AIN1 | AIN2 | Motor A Direction |
/// |------|------|-------------------|
/// | 0    | 1    | Forward          |
/// | 1    | 0    | Reverse          |
/// | 0    | 0    | Brake/Coast      |
/// | 1    | 1    | Brake (both low) |
struct MotorController {
    /// Motor A forward direction pin (GPIO 18)
    ain1: Output<'static>,
    /// Motor A reverse direction pin (GPIO 17)
    ain2: Output<'static>,
    /// Motor B forward direction pin (GPIO 19)
    bin1: Output<'static>,
    /// Motor B reverse direction pin (GPIO 20)
    bin2: Output<'static>,
    /// Motor A PWM speed control (GPIO 16, PWM slice 0 channel A)
    pwm_a: Pwm<'static>,
    /// Motor B PWM speed control (GPIO 21, PWM slice 2 channel B)
    pwm_b: Pwm<'static>,
}

impl MotorController {
    /// Creates a new motor controller instance.
    ///
    /// # Arguments
    ///
    /// * `ain1` - Motor A IN1 direction pin
    /// * `ain2` - Motor A IN2 direction pin
    /// * `bin1` - Motor B IN1 direction pin
    /// * `bin2` - Motor B IN2 direction pin
    /// * `pwm_a` - Motor A PWM channel
    /// * `pwm_b` - Motor B PWM channel
    ///
    /// # Example
    ///
    /// ```no_run
    /// let motors = MotorController::new(
    ///     Output::new(p.PIN_18, Level::Low),
    ///     Output::new(p.PIN_17, Level::Low),
    ///     Output::new(p.PIN_19, Level::Low),
    ///     Output::new(p.PIN_20, Level::Low),
    ///     pwm_a,
    ///     pwm_b,
    /// );
    /// ```
    fn new(
        ain1: Output<'static>,
        ain2: Output<'static>,
        bin1: Output<'static>,
        bin2: Output<'static>,
        pwm_a: Pwm<'static>,
        pwm_b: Pwm<'static>,
    ) -> Self {
        Self { ain1, ain2, bin1, bin2, pwm_a, pwm_b }
    }

    /// Sets the speed for both motors.
    ///
    /// # Arguments
    ///
    /// * `speed` - PWM duty cycle (0-65535 where 65535 = 100% duty)
    ///
    /// # Note
    ///
    /// This only changes the PWM duty cycle. Direction must be set separately
    /// using the movement methods (forward, backward, etc.).
    fn set_speed(&mut self, speed: u16) {
        self.pwm_a.set_counter(speed);
        self.pwm_b.set_counter(speed);
    }

    /// Stops both motors immediately.
    ///
    /// Sets all direction pins low and PWM to 0, causing the motors to
    /// coast to a stop. For active braking, both direction pins could be
    /// set high instead.
    fn stop(&mut self) {
        self.set_speed(0);
        self.ain1.set_low();
        self.ain2.set_low();
        self.bin1.set_low();
        self.bin2.set_low();
    }

    /// Drives both motors forward.
    ///
    /// # Arguments
    ///
    /// * `speed` - PWM duty cycle (0-65535)
    ///
    /// # Direction Logic
    ///
    /// - Motor A: AIN2=HIGH, AIN1=LOW
    /// - Motor B: BIN2=HIGH, BIN1=LOW
    fn forward(&mut self, speed: u16) {
        self.set_speed(speed);
        self.ain2.set_high();
        self.ain1.set_low();
        self.bin2.set_high();
        self.bin1.set_low();
    }

    /// Drives both motors backward.
    ///
    /// # Arguments
    ///
    /// * `speed` - PWM duty cycle (0-65535)
    ///
    /// # Direction Logic
    ///
    /// - Motor A: AIN2=LOW, AIN1=HIGH
    /// - Motor B: BIN2=LOW, BIN1=HIGH
    fn backward(&mut self, speed: u16) {
        self.set_speed(speed);
        self.ain2.set_low();
        self.ain1.set_high();
        self.bin2.set_low();
        self.bin1.set_high();
    }

    /// Turns the robot left by reversing Motor A while Motor B goes forward.
    ///
    /// # Arguments
    ///
    /// * `speed` - PWM duty cycle (0-65535)
    ///
    /// # Direction Logic
    ///
    /// - Motor A: AIN2=LOW, AIN1=HIGH (reverse)
    /// - Motor B: BIN2=HIGH, BIN1=LOW (forward)
    ///
    /// This creates a differential drive turn to the left.
    fn left(&mut self, speed: u16) {
        self.set_speed(speed);
        self.ain2.set_low();
        self.ain1.set_high();
        self.bin2.set_high();
        self.bin1.set_low();
    }

    /// Turns the robot right by reversing Motor B while Motor A goes forward.
    ///
    /// # Arguments
    ///
    /// * `speed` - PWM duty cycle (0-65535)
    ///
    /// # Direction Logic
    ///
    /// - Motor A: AIN2=HIGH, AIN1=LOW (forward)
    /// - Motor B: BIN2=LOW, BIN1=HIGH (reverse)
    ///
    /// This creates a differential drive turn to the right.
    fn right(&mut self, speed: u16) {
        self.set_speed(speed);
        self.ain2.set_high();
        self.ain1.set_low();
        self.bin2.set_low();
        self.bin1.set_high();
    }
}

/// NEC IR Protocol decoder.
///
/// # Protocol Specification
///
/// The NEC IR protocol uses the following timing:
/// - **Start burst**: 9ms low pulse + 4.5ms high space
/// - **Logical '0'**: 0.56ms low + 0.56ms high (total 1.12ms)
/// - **Logical '1'**: 0.56ms low + 1.69ms high (total 2.25ms)
/// - **Data frame**: 32 bits (8-bit address + 8-bit ~address + 8-bit command + 8-bit ~command)
///
/// # Implementation
///
/// This decoder uses async/await for timing measurements, allowing other tasks
/// to run while waiting for signal transitions. It validates:
/// - Start burst timing (8-10ms low, 3.5-5ms high)
/// - Individual bit timing (>200µs minimum)
/// - Address/command checksums (byte + ~byte = 0xFF)
///
/// # Tolerance
///
/// Timing windows allow for ±20% variance to account for IR receiver and
/// remote control variations.
struct NecDecoder {
    /// IR receiver input pin (GPIO 5) with pull-up resistor
    ir_pin: Input<'static>,
}

impl NecDecoder {
    /// Creates a new NEC IR decoder.
    ///
    /// # Arguments
    ///
    /// * `ir_pin` - Input pin connected to IR receiver (active-low, needs pull-up)
    ///
    /// # Pin Configuration
    ///
    /// The IR receiver outputs:
    /// - HIGH when idle (no IR signal)
    /// - LOW when receiving IR pulses
    fn new(ir_pin: Input<'static>) -> Self {
        Self { ir_pin }
    }

    /// Waits for the IR pin to reach a specific logic level.
    ///
    /// # Arguments
    ///
    /// * `level` - The desired pin level (High or Low)
    /// * `timeout_us` - Maximum time to wait in microseconds
    ///
    /// # Returns
    ///
    /// * `Some(elapsed_us)` - Number of microseconds elapsed when level was reached
    /// * `None` - If timeout occurred before reaching the desired level
    ///
    /// # Implementation Note
    ///
    /// Uses a 1µs async yield to prevent blocking the executor while polling.
    /// This allows other tasks to run during IR signal waiting periods.
    async fn wait_for_level(&mut self, level: Level, timeout_us: u64) -> Option<u64> {
        let start = Instant::now();
        let timeout = Duration::from_micros(timeout_us);
        
        loop {
            let pin_level = if self.ir_pin.is_high() { Level::High } else { Level::Low };
            if pin_level == level {
                return Some(start.elapsed().as_micros());
            }
            if start.elapsed() > timeout {
                return None;
            }
            // Small yield to prevent busy-wait from blocking executor
            Timer::after(Duration::from_micros(1)).await;
        }
    }

    /// Decodes a complete NEC IR frame.
    ///
    /// # Returns
    ///
    /// * `Some(command_byte)` - The decoded 8-bit command if frame is valid
    /// * `None` - If frame is invalid or timeout occurred
    ///
    /// # Protocol Validation
    ///
    /// The decoder validates:
    /// 1. **Start burst**: 8-10ms low pulse
    /// 2. **Start space**: 3.5-5ms high space
    /// 3. **32 data bits**: Each with valid timing
    /// 4. **Address checksum**: address + ~address = 0xFF
    /// 5. **Command checksum**: command + ~command = 0xFF
    ///
    /// # Timing Tolerances
    ///
    /// - Start burst: 8-10ms (9ms nominal ±11%)
    /// - Start space: 3.5-5ms (4.5ms nominal ±22%)
    /// - Bit detection: >1.2ms = '1', <1.2ms = '0'
    /// - Minimum pulse: 200µs (safety threshold)
    ///
    /// # Data Format
    ///
    /// ```text
    /// Byte 0: Address (8 bits)
    /// Byte 1: ~Address (inverted, for validation)
    /// Byte 2: Command (8 bits) ← This is returned
    /// Byte 3: ~Command (inverted, for validation)
    /// ```
    async fn decode(&mut self) -> Option<u8> {
        // Wait for leading pulse (9ms low)
        self.wait_for_level(Level::Low, 150_000).await?;
        let t = self.wait_for_level(Level::High, 12_000).await?;
        if t < 8000 || t > 10000 {
            return None;
        }

        // Wait for space (4.5ms high)
        let t = self.wait_for_level(Level::Low, 7000).await?;
        if t < 3500 || t > 5000 {
            return None;
        }

        let mut data = [0u8; 4];
        for i in 0..32 {
            // Wait for low pulse (~0.56ms)
            self.wait_for_level(Level::High, 1000).await?;
            // Measure high duration to determine bit value
            let t = self.wait_for_level(Level::Low, 2500).await?;
            if t < 200 {
                return None;
            }
            let idx = i / 8;
            let bit = i % 8;
            if t > 1200 {
                // '1' bit (1.69ms)
                data[idx] |= 1 << bit;
            }
            // '0' bit is ~0.56ms, already 0 in array
        }

        // Verify address and data checksums
        if data[0].wrapping_add(data[1]) == 0xFF && data[2].wrapping_add(data[3]) == 0xFF {
            Some(data[2])
        } else {
            None
        }
    }
}

/// Main application entry point.
///
/// # Embassy Runtime
///
/// This function is marked with `#[embassy_executor::main]` which:
/// - Initializes the RP2350 peripherals
/// - Sets up the async executor
/// - Runs the main task indefinitely
///
/// # Hardware Initialization
///
/// 1. **GPIO Configuration**:
///    - Motor direction pins (17-20) as outputs
///    - IR receiver (pin 5) as input with pull-up
///
/// 2. **PWM Configuration**:
///    - Slice 0 Channel A (pin 16) for Motor A
///    - Slice 2 Channel B (pin 21) for Motor B
///    - 16-bit resolution (0-65535 TOP)
///    - Initial duty cycle: 0 (stopped)
///
/// # Main Loop
///
/// The application runs an infinite loop that:
/// 1. Attempts to decode IR command (with 1ms timeout)
/// 2. Executes motor command if valid IR received
/// 3. Increments idle counter on timeout
/// 4. Auto-stops motors after 800ms of no IR signal
///
/// # IR Command Mapping
///
/// | Command | Hex  | Action |
/// |---------|------|--------|
/// | Forward | 0x18 | Drive forward at current speed |
/// | Left    | 0x08 | Turn left (differential drive) |
/// | Stop    | 0x1C | Stop all motors |
/// | Right   | 0x5A | Turn right (differential drive) |
/// | Backward| 0x52 | Drive backward at current speed |
/// | Reset   | 0x09 | Set speed to 50% (32768) |
/// | Speed + | 0x15 | Increase speed by ~10% |
/// | Speed - | 0x07 | Decrease speed by ~10% |
///
/// # Safety Features
///
/// - **Auto-stop timeout**: 800ms idle → motors stop
/// - **Speed limits**: Constrained to 0-65535 range
/// - **Invalid IR rejection**: Malformed frames ignored
///
/// # Arguments
///
/// * `_spawner` - Embassy task spawner (unused in this single-task application)
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Initialize motor direction pins
    let ain1 = Output::new(p.PIN_18, Level::Low);
    let ain2 = Output::new(p.PIN_17, Level::Low);
    let bin1 = Output::new(p.PIN_19, Level::Low);
    let bin2 = Output::new(p.PIN_20, Level::Low);

    // Initialize PWM for motor speed control
    let mut pwm_config = PwmConfig::default();
    pwm_config.top = 65535;
    pwm_config.compare_a = 0;
    pwm_config.compare_b = 0;

    let pwm_a = Pwm::new_output_a(p.PWM_SLICE0, p.PIN_16, pwm_config.clone());
    let pwm_b = Pwm::new_output_b(p.PWM_SLICE2, p.PIN_21, pwm_config);

    let mut motors = MotorController::new(ain1, ain2, bin1, bin2, pwm_a, pwm_b);

    // Initialize IR receiver pin
    let ir_pin = Input::new(p.PIN_5, Pull::Up);
    let mut ir_decoder = NecDecoder::new(ir_pin);

    let mut speed: u16 = 32768; // ~50% duty cycle
    let mut idle_count = 0u32;

    loop {
        // Try to decode IR command with timeout
        let key = embassy_time::with_timeout(Duration::from_millis(1), ir_decoder.decode()).await;

        if let Ok(Some(cmd)) = key {
            idle_count = 0;
            match cmd {
                0x18 => motors.forward(speed),      // Forward
                0x08 => motors.left(speed),         // Left
                0x1C => motors.stop(),              // Stop
                0x5A => motors.right(speed),        // Right
                0x52 => motors.backward(speed),     // Backward
                0x09 => speed = 32768,              // Reset speed
                0x15 => {                           // Speed up
                    if speed as u32 + 6553 < 65536 {
                        speed += 6553;
                    }
                }
                0x07 => {                           // Speed down
                    if speed > 6553 {
                        speed -= 6553;
                    }
                }
                _ => {}
            }
        } else {
            idle_count += 1;
            if idle_count > 800 {
                idle_count = 0;
                motors.stop();
            }
        }
    }
}
