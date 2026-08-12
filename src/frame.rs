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

/// Cmd: emergency stop enable/disable (`0x22`, protocol §4.13.1).
pub const CMD_EMERGENCY_STOP: u8 = 0x22;

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

/// Cmd `0x22`: enable (true) or cancel (false) the emergency stop
/// (protocol §4.13.1).
pub fn frame_emergency_stop(enable: bool) -> Vec<u8> {
    with_checksum(vec![
        HEADER as u8,
        (HEADER >> 8) as u8,
        0x07,
        CMD_EMERGENCY_STOP,
        enable as u8,
    ])
}

/// Chassis state decoded from a `0x80` feedback frame.
#[derive(Debug, Clone, PartialEq)]
pub struct ChassisState {
    /// Current control mode (0 = idle, 1 = remote, 2 = serial, 3 = external).
    pub control_mode: u8,
    /// Battery percentage, 0-100%.
    pub battery_percentage: u8,
    /// Battery voltage in 0.1 V units (little endian in frame, per §3.2).
    pub voltage_tenths: u16,
    /// Flags word (e-stop buttons, bumpers, charging, ...).
    pub flags: u16,
    /// Error flags word (driver offline / alarm).
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
/// Requires the frame header, the length byte `0x1A`, `Cmd == 0x80` and a
/// valid trailing checksum (protocol §3.5).
pub fn is_state_frame(frame: &[u8]) -> bool {
    frame.len() == STATE_FRAME_LEN
        && frame[0] == HEADER as u8
        && frame[1] == (HEADER >> 8) as u8
        && frame[2] == STATE_FRAME_LEN as u8
        && frame[3] == CMD_STATE_FEEDBACK
        && checksum_valid(frame)
}

/// Verify the little-endian checksum in the last two bytes against the sum
/// of every other byte (protocol §3.5).
pub fn checksum_valid(frame: &[u8]) -> bool {
    frame.len() >= 2 && {
        let expected = u16::from_le_bytes([frame[frame.len() - 2], frame[frame.len() - 1]]);
        checksum(&frame[..frame.len() - 2]) == expected
    }
}

/// Try to extract one complete, checksum-valid frame from a byte stream.
///
/// Scans for a frame header, then validates the length byte and trailing
/// checksum. Returns the first valid frame; `None` if the buffer holds only a
/// partial or invalid frame. Because the chassis streams state reports at
/// 50 Hz, transports must not wait for the stream to idle — they should hand
/// each complete frame to the caller as soon as it appears and keep any
/// leftover bytes buffered for the next call.
pub fn extract_frame(data: &[u8]) -> Option<&[u8]> {
    let mut i = 0;
    while i + 2 < data.len() {
        if data[i] == HEADER as u8 && data[i + 1] == (HEADER >> 8) as u8 {
            let len = data[i + 2] as usize;
            if len >= 4 && i + len <= data.len() && checksum_valid(&data[i..i + len]) {
                return Some(&data[i..i + len]);
            }
        }
        i += 1;
    }
    None
}

/// Decode a validated `0x80` state frame.
///
/// All multi-byte fields are little endian per protocol §3.2; the expression
/// below fixes a shift-precedence bug in the reference C++ driver that parsed
/// them with `data[7] << 8 + data[6]` (which evaluates as `data[7] << (8 +
/// data[6])`).
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
    fn emergency_stop_frame_matches_layout() {
        // 急停使能 (protocol example: ED DE 07 22 01 F5 01)
        assert_eq!(hex(&frame_emergency_stop(true)), "ED DE 07 22 01 F5 01");
        // 急停取消 (protocol example: ED DE 07 22 00 F4 01)
        assert_eq!(hex(&frame_emergency_stop(false)), "ED DE 07 22 00 F4 01");
    }

    #[test]
    fn state_frame_is_detected_and_decoded() {
        let mut frame = vec![0u8; STATE_FRAME_LEN];
        frame[0] = 0xED;
        frame[1] = 0xDE;
        frame[2] = STATE_FRAME_LEN as u8;
        frame[3] = CMD_STATE_FEEDBACK;
        frame[4] = 2; // control_mode: 2 = serial control (protocol §4.1.3)
        frame[5] = 80; // battery %
        frame[6] = 0x2C; // voltage = 0x012C = 300 -> 30.0 V (LE: 2C 01)
        frame[7] = 0x01;
        frame[8] = 0x00; // flags = 0x0100 (LE: 00 01)
        frame[9] = 0x01;
        frame[10] = 0xAB; // error_flags = 0xCDAB (LE: AB CD)
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
        let sum = checksum(&frame[..STATE_FRAME_LEN - 2]);
        frame[24..26].copy_from_slice(&sum.to_le_bytes());

        let state = decode_state(&frame).expect("state frame should decode");
        assert_eq!(state.control_mode, 2);
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
    fn state_frame_with_bad_checksum_or_length_byte_is_rejected() {
        let mut good = vec![0u8; STATE_FRAME_LEN];
        good[0] = 0xED;
        good[1] = 0xDE;
        good[2] = STATE_FRAME_LEN as u8;
        good[3] = CMD_STATE_FEEDBACK;
        good[5] = 80;
        let sum = checksum(&good[..STATE_FRAME_LEN - 2]);
        good[24..26].copy_from_slice(&sum.to_le_bytes());
        assert!(is_state_frame(&good));

        // corrupt one payload byte -> checksum mismatch
        let mut bad = good.clone();
        bad[5] = 81;
        assert!(!is_state_frame(&bad));
        assert!(decode_state(&bad).is_none());

        // wrong length byte
        let mut wrong_len = good.clone();
        wrong_len[2] = 0x1B;
        assert!(!is_state_frame(&wrong_len));
    }

    #[test]
    fn extract_frame_finds_complete_frames_in_a_stream() {
        let frame = frame_enable_states_upload(true);
        let state = {
            let mut f = vec![0u8; STATE_FRAME_LEN];
            f[0] = HEADER as u8;
            f[1] = (HEADER >> 8) as u8;
            f[2] = STATE_FRAME_LEN as u8;
            f[3] = CMD_STATE_FEEDBACK;
            f[5] = 80;
            let sum = checksum(&f[..STATE_FRAME_LEN - 2]);
            f[24..26].copy_from_slice(&sum.to_le_bytes());
            f
        };

        // two state frames back-to-back in one buffer
        let mut stream = state.clone();
        stream.extend_from_slice(&state);
        let first = extract_frame(&stream).expect("first frame");
        assert_eq!(first.len(), STATE_FRAME_LEN);
        assert_eq!(first, &state[..]);
        let rest = &stream[first.len()..];
        assert_eq!(extract_frame(rest).expect("second frame"), &state[..]);

        // frame straddling a split: bytes arrive in two chunks
        let (a, b) = state.split_at(11);
        let mut split = a.to_vec();
        assert_eq!(extract_frame(&split), None, "partial frame not yet complete");
        split.extend_from_slice(b);
        assert_eq!(extract_frame(&split).expect("joined frame"), &state[..]);

        // frame preceded by garbage bytes is still found
        let mut noisy = vec![0x55, 0xAA, 0x00];
        noisy.extend_from_slice(&state);
        assert_eq!(extract_frame(&noisy).expect("frame after garbage"), &state[..]);

        // enable-upload frame too
        let mut with_upload = frame.clone();
        with_upload.extend_from_slice(&frame);
        assert_eq!(extract_frame(&with_upload).expect("upload frame"), &frame[..]);
    }

    #[test]
    fn checksum_truncates_like_u16() {
        // 258 * 0xFF = 65790, wraps to 65790 - 65536 = 254
        let data = [0xFFu8; 258];
        assert_eq!(checksum(&data), 254);
    }
}
