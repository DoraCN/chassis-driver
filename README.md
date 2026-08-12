# chassis-driver

Rust driver for the **ADORA A2 Pro / A2 Max differential chassis**.

The chassis speaks a simple frame protocol over a serial port
(`/dev/ttyACM0` @ 115200 baud, 8N1) or UDP (`192.168.1.30:1231`). The crate
provides both a library API and a standalone CLI.

## Usage

### As a library

```rust
use chassis_driver::{Chassis, ControlMode, UdpTransport};

let transport = UdpTransport::open("192.168.1.30", 1231, 1231)?;
let mut chassis = Chassis::new(transport);
chassis.init()?;                 // enable the 20 ms state upload
chassis.set_velocity(0.2, 0.0)?; // forward at 0.2 m/s
chassis.set_velocity(0.0, 0.5)?; // rotate at 0.5 rad/s
chassis.stop()?;                 // zero velocity

if let Some(state) = chassis.poll_state()? {
    println!(
        "battery: {}%, vx: {} mm/s, wz: {} (0.001 rad/s)",
        state.battery_percentage, state.vx_mm_s, state.wz_milli_rad_s
    );
}
```

### As a CLI

```sh
# UDP gateway mode
chassis-driver --mode udp status
chassis-driver --mode udp vel 0.2 0.0

# Serial mode
sudo chmod 777 /dev/ttyACM0
chassis-driver --mode serial --serial-port /dev/ttyACM0 status

# Wheel-speed mode (host-side differential solve)
chassis-driver --mode udp --ctrl-mode wheel-speeds vel 0.2 0.5

# Continuous state reporting + keep-alive (default 50 ms tick)
chassis-driver --mode udp watch

# Interactive shell (run `status`, `vel <vx> <wz>`, `vel2 <L> <R>`, `stop`, `quit`)
chassis-driver --mode udp
```

### Motion control & safety

The chassis zeroes the current speed 800 ms after the last speed control
frame (protocol §3.6), so while moving, the driver re-sends the current
velocity command on every 50 ms tick. `vel` / `vel2` either auto-stop after
a duration, or keep the process alive until Ctrl+C:

```sh
# move for 2 seconds, then stop automatically
chassis-driver --mode udp vel 0.2 0.0 --duration 2

# keep moving (speed command re-asserted every 50 ms), stop on Ctrl+C
chassis-driver --mode udp vel 0.1 0.0
```

Safety guarantees:

- **Ctrl+C always stops the chassis** (SIGINT handler sends zero velocity).
- The driver sends zero velocity on `drop` (normal or panicking exit).
- The 800 ms speed timeout is an additional fail-safe: a crashed driver can
  never leave the chassis moving.
- `vel 0 0` / `vel2 0 0` simply exits without waiting.

## Protocol

Frame layout (everything little endian):

```text
| Header (u16 = 0xDEED) | Len (u8) | Cmd (u8) | Data (N) | Check (u16) |
```

`Check` is the sum of all bytes except itself (wrapping `u16`).

| Cmd | Direction | Payload | Meaning |
| --- | --- | --- | --- |
| `0x01` | down | `u8` | enable/disable 20 ms state upload |
| `0x20` | down | `s16 vx (mm/s), s16 wz (0.001 rad/s)` | velocity control |
| `0x21` | down | `s16 left (mm/s), s16 right (mm/s)` | wheel speed control |
| `0x22` | down | `u8` | emergency stop enable/disable |
| `0x80` | up | 20 B | chassis state (see below) |

State feedback (`0x80`, 26 bytes): `control_mode`, `battery_percentage`,
`voltage` (0.1 V), `flags`, `error_flags`, measured `vx` (mm/s), `wz`
(0.001 rad/s), `left`/`right` wheel speeds (mm/s). All multi-byte fields are
little endian (protocol §3.2); received frames are validated against header,
length byte and checksum (protocol §3.5).

## Control modes

- **Velocity (default)**: `linear.x * 1000` and `angular.z * 1000` are sent
  as-is (`0x20`); the chassis controller solves the wheel speeds internally.
- **Wheel speeds**: the host solves the differential kinematics
  `vL = vx - ω·B/2`, `vR = vx + ω·B/2` with wheel base `B = 348 mm` and sends
  the result (`0x21`).

The state upload command and the current speed command are re-sent on every
tick as a keep-alive; the chassis drops the upload when the command is not
repeated, and zeroes the speed 800 ms after the last speed frame (§3.6).

## Safety

- The chassis moves when you send velocity; always be ready to `stop`.
- There is no software speed limiting in the protocol itself; the only
  implicit clamp is the `i16` range of the speed fields
  (±32.7 m/s / ±32.7 rad/s).

## Building for the Jetson Orin (aarch64)

Cross compiling requires a sysroot providing libudev for the target; the
simplest and most reliable option is to build directly on the Jetson:

```sh
cargo build --release
./target/release/chassis-driver --mode serial status
```

See `.cargo/config.toml` for the optional cross-compile setup.

## License

MIT
