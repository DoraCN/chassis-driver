//! Driver for the ADORA A2 Pro / A2 Max differential chassis.
//!
//! The chassis speaks a simple frame protocol over a serial port
//! (`/dev/ttyACM0` @ 115200 baud) or UDP (`192.168.1.30:1231`):
//!
//! ```text
//! | Header (u16 = 0xDEED) | Len (u8) | Cmd (u8) | Data (N) | Check (u16) |
//! ```
//!
//! This crate provides:
//!
//! - [`frame`]: pure framing helpers (checksum, control frames, state parsing)
//! - [`transport`]: serial and UDP transports behind a common trait
//! - [`Chassis`]: the high-level driver (init/keep-alive, velocity and
//!   wheel-speed control, differential kinematics, state feedback)
//!
//! # Example
//!
//! ```
//! use chassis_driver::{Chassis, ControlMode, UdpTransport};
//!
//! # fn run() -> anyhow::Result<()> {
//! let transport = UdpTransport::open("192.168.1.30", 1231, 1231)?;
//! let mut chassis = Chassis::new(transport);
//! chassis.init()?;                  // enable state upload
//! chassis.set_velocity(0.2, 0.0)?;  // forward at 0.2 m/s
//! chassis.set_velocity(0.0, 0.0)?;  // stop
//! # Ok(())
//! # }
//! ```

pub mod chassis;
pub mod frame;
pub mod transport;

pub use chassis::{
    BASE_WIDTH_MM, Chassis, ControlMode, DEFAULT_TICK_MS, SPEED_TIMEOUT_MS,
};
pub use frame::{error_flags, flags, ChassisState};
pub use transport::{
    DEFAULT_SERIAL_BAUD, DEFAULT_SERIAL_PORT, DEFAULT_UDP_IP, DEFAULT_UDP_PORT, SerialTransport,
    Transport, UdpTransport,
};
