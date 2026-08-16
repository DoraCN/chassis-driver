# chassis-driver

Rust driver and CLI for a differential chassis that speaks a framed protocol
over a serial port (`/dev/ttyACM0`, 115200 baud, 8N1).

## Features

- **Velocity control** and **wheel-speed control**, with a keep-alive that
  re-asserts the command every 200 ms (the chassis zeroes the speed 800 ms
  after the last speed frame).
- **50 Hz state feedback**: battery, voltage, measured linear/angular speed,
  per-wheel speed, control mode and status/error flag words.
- **Robust framing**: every frame is validated against the header, length byte
  and checksum, and complete frames are parsed out of the 50 Hz stream as soon
  as they arrive.
- **Safety**: zero velocity on Ctrl+C, on `drop`, and on the 800 ms speed
  timeout as a last resort; explicit warnings when the chassis is e-stopped or
  under remote control.

## Installation

Add the crate to your `Cargo.toml`:

```sh
cargo add chassis_driver
```

### Serial permissions

The serial device must be readable and writable by the current user, and its
device name (`/dev/ttyACM0`, `/dev/ttyACM1`, ...) can change on every boot as
USB devices are enumerated in an arbitrary order.

Use the setup script to pick the chassis's device from a menu once; it writes a
udev rule that pins the device to a stable name:

```sh
sudo scripts/setup-chassis-device.sh --list   # see what is connected
sudo scripts/setup-chassis-device.sh          # choose the chassis device
chassis-driver --serial-port /dev/chassis status
```

The script records the device's `idVendor`/`idProduct`/`serial` in
`/etc/udev/rules.d/99-chassis.rules` and creates a fixed `/dev/chassis`
symlink that no longer depends on the enumeration order. Alternatively, create
the rule by hand (or add your user to the `dialout` group):

```sh
# /etc/udev/rules.d/99-chassis.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d3", ATTRS{serial}=="<serial>", SYMLINK+="chassis", MODE="0666"
```

## Usage

### Library

```rust
use chassis_driver::{Chassis, SerialTransport};

let transport = SerialTransport::open("/dev/ttyACM0", 115200)?;
let mut chassis = Chassis::new(transport);
chassis.init()?;                  // enable the 20 ms state upload

chassis.set_velocity(0.2, 0.0)?;  // forward at 0.2 m/s
chassis.set_velocity(0.0, 0.5)?;  // rotate at 0.5 rad/s
chassis.stop()?;                  // zero velocity

if let Some(state) = chassis.poll_state()? {
    println!(
        "battery: {}%, vx: {} mm/s",
        state.battery_percentage, state.vx_mm_s
    );
}
```

### CLI

```sh
# Read one state report
chassis-driver --serial-port /dev/ttyACM0 status

# Move forward 0.2 m/s for 2 seconds, then stop
chassis-driver --serial-port /dev/ttyACM0 vel 0.2 0.0 --duration 2

# Keep moving until Ctrl+C (command re-asserted every 200 ms)
chassis-driver --serial-port /dev/ttyACM0 vel 0.1 0.0

# Print the measured state every tick while moving (diagnostics)
chassis-driver --serial-port /dev/ttyACM0 vel 0.12 0.0 --duration 4 --report

# Continuous state reporting
chassis-driver --serial-port /dev/ttyACM0 watch
```

## Command-line reference

### Global options

These apply to every invocation and may appear before or after the
subcommand.

| Option | Default | Description |
| --- | --- | --- |
| `--serial-port <PATH>` | `/dev/ttyACM0` | Path of the serial device the chassis is attached to. Typical values are `/dev/ttyACM0`, `/dev/ttyUSB0` or `/dev/ttyS0`. |
| `--serial-baud <RATE>` | `115200` | Serial baud rate in bits per second. The chassis uses `115200` with 8 data bits, no parity and 1 stop bit; only change this for diagnostics (see `baud-scan`). |
| `--ctrl-mode <MODE>` | `velocity` | How velocity commands are issued. `velocity` sends the linear/angular pair straight to the chassis, which solves the wheel speeds internally. `wheel-speeds` solves the differential kinematics on the host (`vL = vx − ω·B/2`, `vR = vx + ω·B/2` with wheel base `B = 348 mm`) and sends left/right wheel speeds. |
| `--debug` | off | Print every frame sent (`>>>`) and received (`<<<`) as hex on stdout. Useful to verify the raw protocol traffic. |
| `--no-init` | off | Skip the startup sequence that enables the state upload. Normally not needed; mainly for diagnosing whether the enable frames are interfering. |

### Subcommands

#### `status`

Read one chassis state report and print it, then exit. Output shows the
control mode, battery percentage, battery voltage, flag words and the measured
linear/angular and left/right wheel speeds. A warning is printed if the chassis
is e-stopped, has a bumper or driver fault, or is under remote control
(`control_mode=1`).

#### `init`

Send the state-upload enable sequence and exit. Useful for leaving the upload
running for another tool.

#### `vel <vx> <wz>`

Set the chassis velocity and keep it there.

- `<vx>` linear velocity in **m/s**. Positive moves forward, negative moves
  backward. Values are converted to mm/s (`vx * 1000`) and clamped to `i16`.
- `<wz>` angular velocity in **rad/s**. Positive rotates counter-clockwise
  (toward the left), negative clockwise. Converted to 0.001 rad/s units and
  clamped to `i16`.

If both values are zero the command is treated as a stop and the process exits
immediately. Otherwise:

- With `--duration <SECS>`: the chassis moves for the given number of seconds,
  then stops automatically.
- Without `--duration`: the process stays alive, re-asserting the command
  every 200 ms, until Ctrl+C stops it.

Options:

| Option | Description |
| --- | --- |
| `--duration <SECS>` | Move for this many seconds, then auto-stop. Without it, keep moving until Ctrl+C. |
| `--report` | Print the measured chassis state on every keep-alive tick while moving. |

#### `vel2 <L> <R>`

Set the left and right wheel speeds directly, in **mm/s**, and keep them
there. `<L>`/`<R>` are signed 16-bit integers; positive is forward. The same
`--duration` and `--report` options apply as for `vel`, and `0 0` stops and
exits.

#### `watch [<MS>]`

Print the chassis state every `<MS>` milliseconds (default `200`) and keep the
state upload alive. Ctrl+C stops. Useful for monitoring while the chassis is
driven by something else.

#### `listen`

Do not send anything; print every byte received as hex. Ctrl+C to stop.
Diagnostic: confirms whether the chassis reports state on its own.

#### `raw <HEX>`

Send one raw frame given as whitespace-separated hex bytes, e.g.
`raw "ED DE 0A 20 64 00 00 00 59 02"`, then print any reply. Diagnostic for
testing hand-written frames.

#### `baud-scan`

Open the serial port at each of several baud rates, send the state-upload
enable frame and report whether the chassis answers. Use when the baud rate is
in doubt.

### Interactive shell

Run `chassis-driver --serial-port /dev/ttyACM0` without a subcommand to get a
prompt:

```text
status                       print one state report
vel <vx> <wz>                set velocity (m/s, rad/s)
vel2 <L> <R>                 set left/right wheel speeds (mm/s)
stop                         zero velocity
quit | exit                  leave the shell (chassis is stopped on drop)
```

## Safety

- The chassis moves when you send velocity — always be ready to `stop`.
- **Ctrl+C always stops the chassis**; the driver also sends zero velocity on
  `drop`, and the chassis's own 800 ms speed timeout stops it if the driver
  crashes.
- There is no software speed limiting in the protocol itself; the only
  implicit clamp is the `i16` range of the speed fields
  (±32.7 m/s / ±32.7 rad/s).

## Protocol

Frame layout, little endian; `Check` is the wrapping `u16` sum of all other
bytes:

```text
| Header (u16 = 0xDEED) | Len (u8) | Cmd (u8) | Data (N) | Check (u16) |
```

| Cmd | Direction | Payload | Meaning |
| --- | --- | --- | --- |
| `0x01` | down | `u8` | enable/disable the 20 ms state upload |
| `0x20` | down | `s16 vx (mm/s), s16 wz (0.001 rad/s)` | velocity control |
| `0x21` | down | `s16 left (mm/s), s16 right (mm/s)` | wheel-speed control |
| `0x22` | down | `u8` | emergency-stop enable/disable |
| `0x80` | up | 26 B | chassis state |

The `0x80` state frame reports `control_mode`, battery percentage, voltage
(0.1 V), a `flags` word, an `error_flags` word, and measured `vx`, `wz`, left
and right wheel speeds. Multi-byte fields are little endian.

`control_mode` decides whether serial velocity commands are honoured:

| Value | Meaning | Serial velocity commands |
| --- | --- | --- |
| `0` | idle | accepted |
| `1` | remote controller in control | **ignored** |
| `2` | serial control | accepted |
| `3` | external control | accepted |

The remote controller must be powered on and set to a mode that allows serial
control, otherwise the chassis reports `control_mode=1` and ignores serial
commands. The CLI warns when it sees this.

`flags` bits: `0` emergency-stop button, `1` remote e-stop, `2` software
e-stop, `3` remote link lost, `4` front bumper, `5` rear bumper, `6` charging.
Any e-stop bit blocks movement; the remote e-stop (`bit 1`) can only be
released on the remote controller itself. `error_flags` bits: `0` driver
offline, `1` driver alarm.

## Building for the Jetson Orin (aarch64)

The simplest and most reliable option is to build directly on the Jetson:

```sh
cargo build --release
./target/release/chassis-driver --serial-port /dev/ttyACM0 status
```

See `.cargo/config.toml` for the optional cross-compile setup (requires a
sysroot providing libudev for the target).

## License

MIT
