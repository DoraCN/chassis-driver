//! Transport layer for a differential chassis: serial or UDP.
//!
//! Mirrors the reference C++ driver: serial on `/dev/ttyACM0` @ 115200 baud
//! (8N1) or a UDP socket bound to the local port `1231` sending to
//! `192.168.1.30:1231`. Receives are non-blocking with a short timeout so the
//! caller can keep ticking without waiting for a reply.

use std::io::{Read, Write};
use std::net::UdpSocket;
use std::time::Duration;

use anyhow::{Context, Result};

/// Default serial device the chassis is attached to.
pub const DEFAULT_SERIAL_PORT: &str = "/dev/ttyACM0";

/// Default baud rate used by the chassis.
pub const DEFAULT_SERIAL_BAUD: u32 = 115_200;

/// Default UDP target IP of the chassis gateway.
pub const DEFAULT_UDP_IP: &str = "192.168.1.30";

/// Default UDP local and target port.
pub const DEFAULT_UDP_PORT: u16 = 1231;

/// How long `recv` waits for incoming bytes before giving up.
const RECV_TIMEOUT: Duration = Duration::from_millis(50);

/// Common transport interface used by [`crate::Chassis`].
pub trait Transport {
    /// Send a raw frame to the chassis.
    fn send(&mut self, frame: &[u8]) -> Result<()>;

    /// Try to read a raw reply, returning `None` on timeout.
    fn recv(&mut self) -> Result<Option<Vec<u8>>>;
}

impl<T: Transport + ?Sized> Transport for Box<T> {
    fn send(&mut self, frame: &[u8]) -> Result<()> {
        (**self).send(frame)
    }

    fn recv(&mut self) -> Result<Option<Vec<u8>>> {
        (**self).recv()
    }
}

/// Serial (UART) transport.
pub struct SerialTransport {
    port: Box<dyn serialport::SerialPort>,
    /// Bytes read but not yet consumed by a complete frame (§3.4): the
    /// chassis streams 26-byte state reports at 50 Hz, so a frame may straddle
    /// two `read`s and several may arrive in one.
    rx: Vec<u8>,
}

impl SerialTransport {
    /// Open the serial port at the given path and baud rate (8N1).
    pub fn open(port: &str, baud_rate: u32) -> Result<Self> {
        let port = serialport::new(port, baud_rate)
            .data_bits(serialport::DataBits::Eight)
            .parity(serialport::Parity::None)
            .stop_bits(serialport::StopBits::One)
            .timeout(RECV_TIMEOUT)
            .open()
            .with_context(|| format!("failed to open serial port {port}"))?;
        Ok(Self {
            port,
            rx: Vec::with_capacity(128),
        })
    }
}

impl Transport for SerialTransport {
    fn send(&mut self, frame: &[u8]) -> Result<()> {
        self.port
            .write_all(frame)
            .with_context(|| "failed to write to serial port")?;
        Ok(())
    }

    fn recv(&mut self) -> Result<Option<Vec<u8>>> {
        if let Some(frame) = crate::frame::extract_frame(&self.rx) {
            let frame = frame.to_vec();
            let consumed = frame.len();
            self.rx.drain(..consumed);
            return Ok(Some(frame));
        }
        let mut tmp = [0u8; 64];
        loop {
            match self.port.read(&mut tmp) {
                Ok(0) => {}
                Ok(n) => self.rx.extend_from_slice(&tmp[..n]),
                Err(e) if e.kind() == std::io::ErrorKind::TimedOut => break,
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
                Err(e) => return Err(e).context("failed to read from serial port"),
            }
            if let Some(frame) = crate::frame::extract_frame(&self.rx) {
                let frame = frame.to_vec();
                let consumed = frame.len();
                self.rx.drain(..consumed);
                return Ok(Some(frame));
            }
        }
        Ok(None)
    }
}

/// UDP transport: bound to a fixed local port, sending to the gateway.
pub struct UdpTransport {
    socket: UdpSocket,
    target: (String, u16),
}

impl UdpTransport {
    /// Bind `0.0.0.0:local_port` and send frames to `target_ip:target_port`.
    pub fn open(target_ip: &str, target_port: u16, local_port: u16) -> Result<Self> {
        let socket =
            UdpSocket::bind(("0.0.0.0", local_port)).context("failed to bind UDP socket")?;
        socket
            .set_read_timeout(Some(RECV_TIMEOUT))
            .context("failed to set UDP read timeout")?;
        Ok(Self {
            socket,
            target: (target_ip.to_string(), target_port),
        })
    }
}

impl Transport for UdpTransport {
    fn send(&mut self, frame: &[u8]) -> Result<()> {
        self.socket
            .send_to(frame, (self.target.0.as_str(), self.target.1))
            .with_context(|| {
                format!(
                    "failed to send UDP frame to {}:{}",
                    self.target.0, self.target.1
                )
            })?;
        Ok(())
    }

    fn recv(&mut self) -> Result<Option<Vec<u8>>> {
        let mut buf = [0u8; 1024];
        match self.socket.recv_from(&mut buf) {
            Ok((n, _addr)) => Ok(Some(buf[..n].to_vec())),
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => Ok(None),
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => Ok(None),
            Err(e) => Err(e).context("failed to receive UDP frame"),
        }
    }
}

/// An in-memory loopback transport, used for tests.
#[cfg(test)]
pub struct LoopbackTransport {
    outbound: std::sync::Mutex<Vec<Vec<u8>>>,
    inbound: std::sync::Mutex<Vec<Vec<u8>>>,
}

#[cfg(test)]
impl LoopbackTransport {
    pub fn new() -> Self {
        Self {
            outbound: std::sync::Mutex::new(Vec::new()),
            inbound: std::sync::Mutex::new(Vec::new()),
        }
    }

    /// Feed a fake chassis reply to the next `recv` call.
    pub fn inject(&self, reply: Vec<u8>) {
        self.inbound.lock().unwrap().push(reply);
    }

    /// All frames sent through this transport so far.
    pub fn sent_frames(&self) -> Vec<Vec<u8>> {
        self.outbound.lock().unwrap().clone()
    }
}

#[cfg(test)]
impl Default for LoopbackTransport {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
impl Transport for LoopbackTransport {
    fn send(&mut self, frame: &[u8]) -> Result<()> {
        self.outbound.lock().unwrap().push(frame.to_vec());
        Ok(())
    }

    fn recv(&mut self) -> Result<Option<Vec<u8>>> {
        Ok(self.inbound.lock().unwrap().pop())
    }
}
