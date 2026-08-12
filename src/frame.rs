//! Frame building, checksum and state parsing for the ADORA A2 Pro / A2 Max
//! chassis protocol.
//!
//! Frame layout (everything little endian, checksum is a plain u16 sum):
//!
//! ```text
//! | Header (u16 = 0xDEED) | Len (u8) | Cmd (u8) | Data (N) | Check (u16) |
//! ```
//!
//! `Len` is the whole frame size in bytes. `Check = Σ bytes[0 .. Len-3]`.

/// Frame header constant (little-endian on the wire: `ED DE`).
pub const HEADER: u16 = 0xDEED;

/// Cmd: enable/disable the 20 ms state upload.
pub const CMD_ENABLE_STATES_UPLOAD: u8 = 0x01;

/// Cmd: velocity control (linear mm/s + angular 0.001 rad/s).
pub const CMD_CONTROL_VELOCITY: u8 = 0x20;

/// Cmd: left/right wheel speed control (mm/s).
pub const CMD_CONTROL_WHEEL_SPEEDS: u8 = 0x21;

/// Cmd: stop (defined by the original firmware, currently unused).
pub const CMD_STOP: u8 = 0x22;

/// Cmd: chassis state feedback frame.
pub const CMD_STATE_FEEDBACK: u8 = 0x80;

/// Size of the state feedback frame.
pub const STATE_FRAME_LEN: usize = 26;

/// Sum of every byte except the trailing checksum, matching the reference
/// C++ driver (`uint16_t` accumulation, wrapping on overflow).
pub fn checksum(frame_without_check: &[u8]) -> u16 {
    frame_without_check
        .iter()
        .fold(0u16, |acc, &b| acc.wrapping_add(b as u16))
}

/// Append the little-endian checksum to a frame body.
fn with_checksum(mut frame: Vec<u8>) -> Vec<u8> {
    let sum = checksum(&frame);
    frame.extend_from_slice(&sum.to_le_bytes());
    frame
}

/// Cmd `0x01`: enable (true) or disable (false) the 20 ms state upload.
pub fn frame_enable_states_upload(enable: bool) -> Vec<u8> {
    with_checksum(vec![
        HEADER as u8,
        (HEADER >> 8) as u8,
        0x07,
        CMD_ENABLE_STATES_UPLOAD,
        enable as u8,
    ])
}

/// Cmd `0x20`: set linear velocity (`mm/s`) and angular velocity
/// (`0.001 rad/s`), both signed 16-bit little endian.
pub fn frame_control_velocity(vx_mm_s: i16, wz_milli_rad_s: i16) -> Vec<u8> {
    let mut frame = vec![
        HEADER as u8,
        (HEADER >> 8) as u8,
        0x0A,
        CMD_CONTROL_VELOCITY,
    ];
    frame.extend_from_slice(&vx_mm_s.to_le_bytes());
    frame.extend_from_slice(&wz_milli_rad_s.to_le_bytes());
    with_checksum(frame)
}

/// Cmd `0x21`: set left and right wheel speeds in mm/s (signed 16-bit LE).
pub fn frame_control_wheel_speeds(left_mm_s: i16, right_mm_s: i16) -> Vec<u8> {
    let mut frame = vec![
        HEADER as u8,
        (HEADER >> 8) as u8,
        0x0A,
        CMD_CONTROL_WHEEL_SPEEDS,
    ];
    frame.extend_from_slice(&left_mm_s.to_le_bytes());
    frame.extend_from_slice(&right_mm_s.to_le_bytes());
    with_checksum(frame)
}

/// Cmd `0x22`: stop command frame (kept for completeness, unused by the
/// original firmware flow).
pub fn frame_stop(enable: bool) -> Vec<u8> {
    with_checksum(vec![
        HEADER as u8,
        (HEADER >> 8) as u8,
        0x07,
        CMD_STOP,
        enable as u8,
    ])
}

/// Chassis state decoded from a `0x80` feedback frame.
#[derive(Debug, Clone, PartialEq)]
pub struct ChassisState {
    /// Current control mode (1 = velocity, 2 = wheel speeds).
    pub control_mode: u8,
    /// Battery percentage, 0-100%.
    pub battery_percentage: u8,
    /// Battery voltage in 0.1 V units (big endian in frame).
    pub voltage_tenths: u16,
    /// Flags word.
    pub flags: u16,
    /// Error flags word.
    pub error_flags: u16,
    /// Measured linear velocity, mm/s.
    pub vx_mm_s: i16,
    /// Measured angular velocity, 0.001 rad/s.
    pub wz_milli_rad_s: i16,
    /// Measured left wheel speed, mm/s.
    pub left_mm_s: i16,
    /// Measured right wheel speed, mm/s.
    pub right_mm_s: i16,
}

/// Check whether `frame` looks like a chassis state feedback frame.
///
/// Requires the frame header, `Cmd == 0x80` and the expected length.
pub fn is_state_frame(frame: &[u8]) -> bool {
    frame.len() == STATE_FRAME_LEN
        && frame[0] == HEADER as u8
        && frame[1] == (HEADER >> 8) as u8
        && frame[3] == CMD_STATE_FEEDBACK
}

/// Decode a validated `0x80` state frame.
///
/// Fields `voltage`, `flags` and `error_flags` are big endian in the frame
/// (this fixes a shift-precedence bug in the reference C++ driver that
/// parsed them with `data[7] << 8 + data[6]`).
pub fn decode_state(frame: &[u8]) -> Option<ChassisState> {
    if !is_state_frame(frame) {
        return None;
    }
    let vx_mm_s = i16::from_le_bytes([frame[12], frame[13]]);
    let wz_milli_rad_s = i16::from_le_bytes([frame[14], frame[15]]);
    let left_mm_s = i16::from_le_bytes([frame[16], frame[17]]);
    let right_mm_s = i16::from_le_bytes([frame[18], frame[19]]);
    Some(ChassisState {
        control_mode: frame[4],
        battery_percentage: frame[5],
        voltage_tenths: ((frame[7] as u16) << 8) | frame[6] as u16,
        flags: ((frame[9] as u16) << 8) | frame[8] as u16,
        error_flags: ((frame[11] as u16) << 8) | frame[10] as u16,
        vx_mm_s,
        wz_milli_rad_s,
        left_mm_s,
        right_mm_s,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hex(bytes: &[u8]) -> String {
        bytes
            .iter()
            .map(|b| format!("{b:02X}"))
            .collect::<Vec<_>>()
            .join(" ")
    }

    #[test]
    fn enable_upload_frame_matches_layout() {
        // ED DE 07 01 01 + check(0xED+0xDE+0x07+0x01+0x01 = 0x1D4)
        assert_eq!(
            hex(&frame_enable_states_upload(true)),
            "ED DE 07 01 01 D4 01"
        );
        // ED DE 07 01 00 + check(0xED+0xDE+0x07+0x01 = 0x1D3)
        assert_eq!(
            hex(&frame_enable_states_upload(false)),
            "ED DE 07 01 00 D3 01"
        );
    }

    #[test]
    fn velocity_frame_matches_layout() {
        // ED DE 0A 20 E8 03 C8 00
        // check = 0xED+0xDE+0x0A+0x20+0xE8+0x03+0xC8 = 0x3A8
        assert_eq!(
            hex(&frame_control_velocity(1000, 200)),
            "ED DE 0A 20 E8 03 C8 00 A8 03"
        );
        // negative velocities, two's complement
        assert_eq!(
            hex(&frame_control_velocity(-1000, -200)),
            "ED DE 0A 20 18 FC 38 FF 40 04"
        );
        // zero velocities
        assert_eq!(
            hex(&frame_control_velocity(0, 0)),
            "ED DE 0A 20 00 00 00 00 F5 01"
        );
    }

    #[test]
    fn wheel_speed_frame_matches_layout() {
        // ED DE 0A 21 90 01 70 00, check = 0xED+0xDE+0x0A+0x21+0x90+0x01+0x70 = 0x2F7
        assert_eq!(
            hex(&frame_control_wheel_speeds(400, 112)),
            "ED DE 0A 21 90 01 70 00 F7 02"
        );
    }

    #[test]
    fn stop_frame_matches_layout() {
        assert_eq!(hex(&frame_stop(true)), "ED DE 07 22 01 F5 01");
    }

    #[test]
    fn state_frame_is_detected_and_decoded() {
        let mut frame = vec![0u8; STATE_FRAME_LEN];
        frame[0] = 0xED;
        frame[1] = 0xDE;
        frame[2] = STATE_FRAME_LEN as u8;
        frame[3] = CMD_STATE_FEEDBACK;
        frame[4] = 1; // control_mode
        frame[5] = 80; // battery %
        frame[6] = 0x2C; // voltage = 0x012C = 300 -> 30.0 V (BE)
        frame[7] = 0x01;
        frame[8] = 0x00; // flags = 0x0100
        frame[9] = 0x01;
        frame[10] = 0xAB; // error_flags = 0xCDAB
        frame[11] = 0xCD;
        frame[12] = 0xE8; // vx = 1000
        frame[13] = 0x03;
        frame[14] = 0xC8; // wz = 200
        frame[15] = 0x00;
        frame[16] = 0x90; // vl = 400
        frame[17] = 0x01;
        frame[18] = 0x70; // vr = 112
        frame[19] = 0x00;
        // bytes 20-23 reserved

        let state = decode_state(&frame).expect("state frame should decode");
        assert_eq!(state.control_mode, 1);
        assert_eq!(state.battery_percentage, 80);
        assert_eq!(state.voltage_tenths, 300);
        assert_eq!(state.flags, 0x0100);
        assert_eq!(state.error_flags, 0xCDAB);
        assert_eq!(state.vx_mm_s, 1000);
        assert_eq!(state.wz_milli_rad_s, 200);
        assert_eq!(state.left_mm_s, 400);
        assert_eq!(state.right_mm_s, 112);

        assert!(!is_state_frame(&frame[..25]));
        assert!(!is_state_frame(&[0u8; 26]));
    }

    #[test]
    fn checksum_truncates_like_u16() {
        // 258 * 0xFF = 65790, wraps to 65790 - 65536 = 254
        let data = [0xFFu8; 258];
        assert_eq!(checksum(&data), 254);
    }
}
