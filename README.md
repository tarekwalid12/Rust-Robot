![Rust Robot](https://github.com/mytechnotalent/Rust-Robot/blob/main/Rust-Robot.png?raw=true)

## FREE Reverse Engineering Self-Study Course [HERE](https://github.com/mytechnotalent/Reverse-Engineering-Tutorial)

<br>

# Rust Robot

Async robot firmware for Waveshare Pico2Go (RP2350-Plus) in Embedded Rust with Embassy; leverages RP2350 for reliable motor control plus integrated IR remote support and robust capabilities.

## Required Hardware

- **[Waveshare Pico2Go Mobile Robot (Amazon B0FFH15ZW4)](https://www.amazon.com/dp/B0FFH15ZW4)**  
    - Mobile robot platform based on the RP2350-Plus microcontroller, compatible with Raspberry Pi Pico series.  
    - Features: Self-driving, remote control, includes RP2350-Plus control board, rich Wiki resources, and expansion options.  
    - Ideal for embedded robotics, motor control, and IR remote experiments.

- **[PAOWANG 14500 Rechargeable Battery 3.7V 2500mAh + Charger (Amazon B0CK1PWMTT)](https://www.amazon.com/dp/B0CK1PWMTT)**  
    - 4-pack of 14500 lithium-ion rechargeable batteries (button top) with charger.  
    - 3.7V, 2500mAh high capacity, suitable for powering the Pico2Go robot and similar embedded projects.  
    - Rechargeable, long life, and includes charger for convenience.


## 🚀 Quick Start

```bash
# Put your RP2350 in BOOTSEL mode (hold button while plugging USB)
./quick-flash.sh
```

That's it! The script will build and flash automatically.

## What's Included

- **Motor Control**: PWM-based speed control with direction pins
- **NEC IR Protocol Decoder**: Async IR remote receiver 
- **Embassy Async Runtime**: Clean async/await implementation for RP2350

## Hardware Pin Mapping

- **PWMA**: GPIO 16
- **AIN2**: GPIO 17  
- **AIN1**: GPIO 18
- **BIN1**: GPIO 19
- **BIN2**: GPIO 20
- **PWMB**: GPIO 21
- **IR Receiver**: GPIO 5

## Building

This project uses Embassy from git (main branch) since RP2350 support is very recent.

```bash
# Install the ARM Cortex-M33 target (already done if you followed setup)
rustup target add thumbv8m.main-none-eabihf

# Build
cargo build --release
```

## Flashing to RP2350

### Method 1: BOOTSEL Mode (Recommended)

Use the included flash script for devices in BOOTSEL mode:

```bash
# 1. Hold BOOTSEL button on RP2350 and plug in USB
# 2. Run the flash script
./quick-flash.sh
```

The script will:
- Build the release binary
- Convert ELF to UF2 format
- Flash using picotool (or copy to mounted volume)
- Auto-reset the device

### Method 2: Manual UF2 Copy

```bash
cargo build --release

# Install elf2uf2-rs if not already installed
cargo install elf2uf2-rs --locked

# Convert to UF2
elf2uf2-rs target/thumbv8m.main-none-eabihf/release/robot-embassy \
           target/thumbv8m.main-none-eabihf/release/robot-embassy.uf2

# Copy to mounted RP2350 volume
cp target/thumbv8m.main-none-eabihf/release/robot-embassy.uf2 /Volumes/RP2350/
```

### Method 3: Debug Probe (probe-rs)

If you have a debug probe connected:

```bash
# Install probe-rs
cargo install probe-rs-tools --locked

# Uncomment the runner line in .cargo/config.toml
# Then run:
cargo run --release
```

## Dependencies Note

The `Cargo.toml` uses git dependencies for Embassy crates to get the latest RP2350 support.  
Once RP2350 support is published to crates.io, you can switch to versioned dependencies.

## Key Differences from C Version

1. **Async/Await**: IR decoding runs asynchronously using Embassy timers
2. **Type Safety**: Rust's type system prevents many common embedded bugs
3. **No Blocking Loops**: Uses `embassy_time::with_timeout` for clean timeout handling
4. **PWM Abstraction**: Higher-level PWM API from embassy-rp

## IR Remote Commands

- `0x18`: Forward
- `0x08`: Left  
- `0x1C`: Stop
- `0x5A`: Right
- `0x52`: Backward
- `0x09`: Reset speed to 50%
- `0x15`: Increase speed
- `0x07`: Decrease speed

Auto-stop after 800ms of no IR signal.

---

## License

[MIT](https://github.com/mytechnotalent/Rust-Robot/blob/main/LICENSE)
