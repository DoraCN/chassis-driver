//! Core chassis driver for the ADORA A2 Pro / A2 Max.
//!
//! Behaviour mirrors the reference C++ node (`adora_chassis_a2pro_dora_node.cc`):
//!
//! - mode 1 (velocity, default): `linear.x * 1000` mm/s and
//!   `angular.z * 1000` (0.001 rad/s) are sent straight to the chassis
//!   controller, which solves the wheel speeds internally.
//! - mode 2 (wheel speeds): the host solves the differential kinematics
//!   `vL = vx - ω·B/2`, `vR = vx + ω·B/2` with `B = 348 mm` and sends the
//!   left/right wheel speeds in mm/s.
//! - state upload is enabled once on init and re-asserted on every tick
//!   (keep-alive), matching the reference driver.

use anyhow::{Context, Result};

use crate::frame::{
    ChassisState, decode_state, frame_control_velocity, frame_control_wheel_speeds,
    frame_enable_states_upload,
};
use crate::transport::Transport;

/// Distance between the left and right wheels, in millimetres.
pub const BASE_WIDTH_MM: f64 = 348.0;

/// Milliseconds between the reference driver's keep-alive ticks.
pub const DEFAULT_TICK_MS: u64 = 50;

/// Control mode of the chassis.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ControlMode {
    /// Velocity mode (`0x20` frame): linear + angular sent as-is.
    #[default]
    Velocity = 1,
    /// Wheel-speed mode (`0x21` frame): host-side differential solve.
    WheelSpeeds = 2,
}

/// Driver for the ADORA A2 Pro / A2 Max differential chassis.
pub struct Chassis<T: Transport> {
    transport: T,
    control_mode: ControlMode,
    base_width_mm: f64,
    state: Option<ChassisState>,
}

impl<T: Transport> Chassis<T> {
    /// Create a driver around an already-open transport.
    pub fn new(transport: T) -> Self {
        Self {
            transport,
            control_mode: ControlMode::default(),
            base_width_mm: BASE_WIDTH_MM,
            state: None,
        }
    }

    /// Select the control mode used by [`Self::set_velocity`].
    pub fn with_control_mode(mut self, mode: ControlMode) -> Self {
        self.control_mode = mode;
        self
    }

    /// Override the wheel base used for the differential solve (default 348 mm).
    pub fn with_base_width_mm(mut self, mm: f64) -> Self {
        self.base_width_mm = mm;
        self
    }

    /// The most recently decoded chassis state.
    pub fn state(&self) -> Option<&ChassisState> {
        self.state.as_ref()
    }

    /// Enable (or disable) the 20 ms state upload.
    pub fn enable_states_upload(&mut self, enable: bool) -> Result<()> {
        self.transport
            .send(&frame_enable_states_upload(enable))
            .context("failed to send states upload frame")
    }

    /// Initialise the chassis: enable state upload, matching the reference
    /// driver's startup sequence.
    pub fn init(&mut self) -> Result<()> {
        self.enable_states_upload(true)?;
        std::thread::sleep(std::time::Duration::from_millis(200));
        self.enable_states_upload(true)?;
        Ok(())
    }

    /// Keep-alive: re-assert state upload (the chassis drops the upload if
    /// the command is not repeated).
    pub fn keep_alive(&mut self) -> Result<()> {
        self.enable_states_upload(true)
    }

    /// Set the chassis velocity directly (`0x20` frame).
    ///
    /// `vx` is linear velocity in m/s, `wz` is angular velocity in rad/s.
    /// Units are converted to mm/s and 0.001 rad/s with `i16` saturation.
    pub fn set_velocity_direct(&mut self, vx_m_s: f64, wz_rad_s: f64) -> Result<()> {
        let vx_mm_s = (vx_m_s * 1000.0) as i16;
        let wz_milli_rad_s = (wz_rad_s * 1000.0) as i16;
        self.transport
            .send(&frame_control_velocity(vx_mm_s, wz_milli_rad_s))
            .context("failed to send velocity frame")
    }

    /// Set left/right wheel speeds in mm/s (`0x21` frame).
    pub fn set_wheel_speeds_mm_s(&mut self, left_mm_s: i16, right_mm_s: i16) -> Result<()> {
        self.transport
            .send(&frame_control_wheel_speeds(left_mm_s, right_mm_s))
            .context("failed to send wheel speed frame")
    }

    /// Set the chassis velocity according to the configured control mode.
    ///
    /// In [`ControlMode::Velocity`] the command goes out untouched; in
    /// [`ControlMode::WheelSpeeds`] the differential kinematics are solved
    /// with the configured wheel base first.
    pub fn set_velocity(&mut self, vx_m_s: f64, wz_rad_s: f64) -> Result<()> {
        match self.control_mode {
            ControlMode::Velocity => self.set_velocity_direct(vx_m_s, wz_rad_s),
            ControlMode::WheelSpeeds => {
                let vx_mm_s = vx_m_s * 1000.0;
                let left = (vx_mm_s - wz_rad_s * self.base_width_mm / 2.0) as i16;
                let right = (vx_mm_s + wz_rad_s * self.base_width_mm / 2.0) as i16;
                self.set_wheel_speeds_mm_s(left, right)
            }
        }
    }

    /// Stop the chassis (send zero velocity).
    pub fn stop(&mut self) -> Result<()> {
        self.set_velocity(0.0, 0.0)
    }

    /// Try to read one chassis state reply, if available.
    ///
    /// Returns `Ok(None)` when no valid `0x80` frame is pending. On success
    /// the state is cached in [`Self::state`].
    pub fn poll_state(&mut self) -> Result<Option<ChassisState>> {
        let Some(reply) = self.transport.recv()? else {
            return Ok(None);
        };
        if let Some(state) = decode_state(&reply) {
            self.state = Some(state.clone());
            return Ok(Some(state));
        }
        Ok(None)
    }
}

impl<T: Transport> Drop for Chassis<T> {
    /// Safety net: send zero velocity when the driver is dropped, so a
    /// normal (or panicking) exit never leaves the chassis moving.
    fn drop(&mut self) {
        let _ = self.stop();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transport::LoopbackTransport;

    fn state_frame(vx_mm_s: i16, wz_milli_rad_s: i16) -> Vec<u8> {
        let mut frame = vec![0u8; crate::frame::STATE_FRAME_LEN];
        frame[0] = 0xED;
        frame[1] = 0xDE;
        frame[2] = crate::frame::STATE_FRAME_LEN as u8;
        frame[3] = crate::frame::CMD_STATE_FEEDBACK;
        frame[4] = 1;
        frame[5] = 100;
        frame[6..8].copy_from_slice(&300u16.to_be_bytes());
        frame[12..14].copy_from_slice(&vx_mm_s.to_le_bytes());
        frame[14..16].copy_from_slice(&wz_milli_rad_s.to_le_bytes());
        frame
    }

    #[test]
    fn init_sends_enable_frames() {
        let t = LoopbackTransport::new();
        let mut chassis = Chassis::new(t);
        chassis.init().unwrap();
        let frames = chassis.transport.sent_frames();
        assert_eq!(frames.len(), 2);
        assert_eq!(frames[0], crate::frame::frame_enable_states_upload(true));
        assert_eq!(frames[1], crate::frame::frame_enable_states_upload(true));
    }

    #[test]
    fn velocity_mode_converts_units() {
        let t = LoopbackTransport::new();
        let mut chassis = Chassis::new(t);

        chassis.set_velocity(1.0, 0.5).unwrap();
        // 1.0 m/s -> 1000 mm/s, 0.5 rad/s -> 500 (0.001 rad/s)
        assert_eq!(
            chassis.transport.sent_frames().last().unwrap(),
            &crate::frame::frame_control_velocity(1000, 500)
        );

        // negative + i16 saturation safety
        chassis.set_velocity(-0.2, 0.0).unwrap();
        assert_eq!(
            chassis.transport.sent_frames().last().unwrap(),
            &crate::frame::frame_control_velocity(-200, 0)
        );
    }

    #[test]
    fn wheel_mode_solves_differential_kinematics() {
        let t = LoopbackTransport::new();
        let mut chassis = Chassis::new(t).with_control_mode(ControlMode::WheelSpeeds);

        // vx = 0.2 m/s, wz = 0.5 rad/s, B = 348 mm
        // vL = 200 - 0.5*348/2 = 200 - 87 = 113 mm/s
        // vR = 200 + 0.5*348/2 = 200 + 87 = 287 mm/s
        chassis.set_velocity(0.2, 0.5).unwrap();
        assert_eq!(
            chassis.transport.sent_frames().last().unwrap(),
            &crate::frame::frame_control_wheel_speeds(113, 287)
        );

        // pure rotation: vL = -87, vR = 87
        chassis.set_velocity(0.0, 0.5).unwrap();
        assert_eq!(
            chassis.transport.sent_frames().last().unwrap(),
            &crate::frame::frame_control_wheel_speeds(-87, 87)
        );
    }

    #[test]
    fn stop_sends_zero_velocity() {
        let t = LoopbackTransport::new();
        let mut chassis = Chassis::new(t);
        chassis.set_velocity(1.0, 1.0).unwrap();
        chassis.stop().unwrap();
        assert_eq!(
            chassis.transport.sent_frames().last().unwrap(),
            &crate::frame::frame_control_velocity(0, 0)
        );
    }

    #[test]
    fn poll_state_decodes_and_caches() {
        let t = LoopbackTransport::new();
        // loopback pops last-in-first-out: garbage is consumed first
        t.inject(state_frame(1000, 200));
        t.inject(vec![0x00; 26]);
        let mut chassis = Chassis::new(t);

        // garbage is ignored
        assert_eq!(chassis.poll_state().unwrap(), None);

        let state = chassis.poll_state().unwrap().expect("state present");
        assert_eq!(state.vx_mm_s, 1000);
        assert_eq!(state.wz_milli_rad_s, 200);
        assert_eq!(chassis.state().unwrap().battery_percentage, 100);
    }
}
