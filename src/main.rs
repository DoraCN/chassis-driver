//! Command-line interface for the ADORA A2 Pro / A2 Max chassis.
//!
//! Standalone replacement for the C++ `adora_chassis_a2pro_dora_node`:
//! enables state upload, sends velocity commands and reports feedback.

use std::io::{self, BufRead, Write};

use anyhow::{Context, Result};
use clap::{Parser, Subcommand, ValueEnum};

use chassis_driver::{
    Chassis, ControlMode, DEFAULT_SERIAL_BAUD, DEFAULT_SERIAL_PORT, DEFAULT_UDP_IP,
    DEFAULT_UDP_PORT, SerialTransport, Transport, UdpTransport,
};

#[derive(Parser, Debug)]
#[command(
    name = "chassis-driver",
    version,
    about = "ADORA A2 Pro / A2 Max chassis driver"
)]
struct Cli {
    /// Communication mode: serial port or UDP gateway.
    #[arg(long, value_enum, default_value_t = Mode::Serial)]
    mode: Mode,

    /// Serial device path (mode=serial).
    #[arg(long, default_value = DEFAULT_SERIAL_PORT)]
    serial_port: String,

    /// Serial baud rate (mode=serial).
    #[arg(long, default_value_t = DEFAULT_SERIAL_BAUD)]
    serial_baud: u32,

    /// UDP target IP (mode=udp).
    #[arg(long, default_value = DEFAULT_UDP_IP)]
    udp_ip: String,

    /// UDP local/target port (mode=udp).
    #[arg(long, default_value_t = DEFAULT_UDP_PORT)]
    udp_port: u16,

    /// Control mode: 1 = velocity (chassis solves wheels), 2 = wheel speeds.
    #[arg(long, value_enum, default_value_t = CtrlMode::Velocity)]
    ctrl_mode: CtrlMode,

    /// Skip the init sequence (state upload enable).
    #[arg(long)]
    no_init: bool,

    /// Print every frame sent and received as hex (diagnostics).
    #[arg(long)]
    debug: bool,

    /// Run `command` once and exit; without it an interactive loop starts.
    #[command(subcommand)]
    command: Option<Command>,
}

/// Wraps a transport and prints every frame in hex for diagnostics.
struct DebugTransport<T: Transport> {
    inner: T,
}

impl<T: Transport> DebugTransport<T> {
    fn new(inner: T) -> Self {
        Self { inner }
    }
}

impl<T: Transport> Transport for DebugTransport<T> {
    fn send(&mut self, frame: &[u8]) -> Result<()> {
        println!(">>> {}", hex(frame));
        self.inner.send(frame)
    }

    fn recv(&mut self) -> Result<Option<Vec<u8>>> {
        match self.inner.recv()? {
            Some(data) => {
                println!("<<< {}", hex(&data));
                Ok(Some(data))
            }
            None => Ok(None),
        }
    }
}

fn hex(bytes: &[u8]) -> String {
    bytes
        .iter()
        .map(|b| format!("{b:02X}"))
        .collect::<Vec<_>>()
        .join(" ")
}

#[derive(Debug, Clone, Copy, ValueEnum)]
enum Mode {
    Serial,
    Udp,
}

#[derive(Debug, Clone, Copy, ValueEnum)]
enum CtrlMode {
    Velocity,
    WheelSpeeds,
}

impl From<CtrlMode> for ControlMode {
    fn from(m: CtrlMode) -> Self {
        match m {
            CtrlMode::Velocity => ControlMode::Velocity,
            CtrlMode::WheelSpeeds => ControlMode::WheelSpeeds,
        }
    }
}

#[derive(Debug, Subcommand)]
enum Command {
    /// Enable the state upload and exit.
    Init,
    /// Read one chassis state report and print it.
    Status,
    /// Set velocity: forward <vx> m/s and rotation <wz> rad/s.
    /// Without --duration the process stays alive until Ctrl+C (auto-stop).
    Vel {
        vx: f64,
        wz: f64,
        /// Auto-stop after this many seconds.
        #[arg(long)]
        duration: Option<f64>,
    },
    /// Set left/right wheel speeds in mm/s directly (mode 2).
    Vel2 {
        left: i16,
        right: i16,
        /// Auto-stop after this many seconds.
        #[arg(long)]
        duration: Option<f64>,
    },
    /// Print chassis state every <ms> (default 50) and keep the upload alive.
    Watch { ms: Option<u64> },
}

fn open_transport(cli: &Cli) -> Result<Box<dyn Transport>> {
    match cli.mode {
        Mode::Serial => Ok(Box::new(SerialTransport::open(
            &cli.serial_port,
            cli.serial_baud,
        )?)),
        Mode::Udp => Ok(Box::new(UdpTransport::open(
            &cli.udp_ip,
            cli.udp_port,
            cli.udp_port,
        )?)),
    }
}

fn print_state(state: &chassis_driver::ChassisState) {
    println!(
        "control_mode={} battery={}% voltage={:.1}V flags=0x{:04X} errors=0x{:04X} vx={}mm/s wz={}(0.001rad/s) L={}mm/s R={}mm/s",
        state.control_mode,
        state.battery_percentage,
        state.voltage_tenths as f64 / 10.0,
        state.flags,
        state.error_flags,
        state.vx_mm_s,
        state.wz_milli_rad_s,
        state.left_mm_s,
        state.right_mm_s,
    );
}

fn read_once(chassis: &mut Chassis<Box<dyn Transport>>) -> Result<()> {
    match chassis.poll_state()? {
        Some(state) => print_state(&state),
        None => println!("status: <no reply>"),
    }
    Ok(())
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    let transport = open_transport(&cli)?;
    let transport: Box<dyn Transport> = if cli.debug {
        Box::new(DebugTransport::new(transport))
    } else {
        transport
    };
    let mut chassis = Chassis::new(transport).with_control_mode(cli.ctrl_mode.into());

    if !cli.no_init {
        chassis.init()?;
    }

    match &cli.command {
        Some(Command::Init) => {}
        Some(Command::Status) => read_once(&mut chassis)?,
        Some(Command::Vel { vx, wz, duration }) => {
            chassis.set_velocity(*vx, *wz)?;
            println!("set velocity: vx={vx} m/s, wz={wz} rad/s");
            // zero velocity means stop: nothing to keep alive
            if *vx == 0.0 && *wz == 0.0 {
                return Ok(());
            }
            match duration {
                Some(secs) => {
                    println!("moving for {secs}s, then auto-stop");
                    std::thread::sleep(std::time::Duration::from_secs_f64(*secs));
                    chassis.stop()?;
                    println!("stopped");
                }
                None => {
                    println!("chassis moving. Press Ctrl+C to stop.");
                    wait_for_ctrl_c()?;
                    chassis.stop()?;
                    println!("stopped");
                }
            }
        }
        Some(Command::Vel2 {
            left,
            right,
            duration,
        }) => {
            chassis.set_wheel_speeds_mm_s(*left, *right)?;
            println!("set wheel speeds: L={left} mm/s, R={right} mm/s");
            if *left == 0 && *right == 0 {
                return Ok(());
            }
            match duration {
                Some(secs) => {
                    println!("moving for {secs}s, then auto-stop");
                    std::thread::sleep(std::time::Duration::from_secs_f64(*secs));
                    chassis.stop()?;
                    println!("stopped");
                }
                None => {
                    println!("chassis moving. Press Ctrl+C to stop.");
                    wait_for_ctrl_c()?;
                    chassis.stop()?;
                    println!("stopped");
                }
            }
        }
        Some(Command::Watch { ms }) => {
            let interval = ms.unwrap_or(chassis_driver::DEFAULT_TICK_MS);
            loop {
                if STOPPED.load(std::sync::atomic::Ordering::Relaxed) {
                    println!("received stop signal, stopping chassis");
                    break;
                }
                chassis.keep_alive()?;
                read_once(&mut chassis)?;
                std::thread::sleep(std::time::Duration::from_millis(interval));
            }
            chassis.stop()?;
        }
        None => interactive(&mut chassis)?,
    }
    Ok(())
}

static STOPPED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

/// Install a Ctrl+C handler; when triggered, the main flow stops the chassis.
fn wait_for_ctrl_c() -> Result<()> {
    ctrlc::set_handler(move || {
        STOPPED.store(true, std::sync::atomic::Ordering::Relaxed);
    })?;
    while !STOPPED.load(std::sync::atomic::Ordering::Relaxed) {
        std::thread::sleep(std::time::Duration::from_millis(100));
    }
    Ok(())
}

fn interactive(chassis: &mut Chassis<Box<dyn Transport>>) -> Result<()> {
    let stdin = io::stdin();
    let mut stdout = io::stdout();
    println!(
        "chassis-driver interactive shell. Commands: status | vel <vx> <wz> | vel2 <L> <R> | stop | quit"
    );
    loop {
        print!("> ");
        stdout.flush()?;
        let mut line = String::new();
        if stdin.lock().read_line(&mut line)? == 0 {
            break;
        }
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        match line {
            "status" => {
                chassis.keep_alive()?;
                read_once(chassis)?;
            }
            "stop" => {
                chassis.stop()?;
                println!("chassis stopped");
            }
            "quit" | "exit" => break,
            _ if line.starts_with("vel ") => {
                let mut parts = line.split_whitespace().skip(1);
                let vx: f64 = parts.next().context("usage: vel <vx> <wz>")?.parse()?;
                let wz: f64 = parts.next().context("usage: vel <vx> <wz>")?.parse()?;
                chassis.set_velocity(vx, wz)?;
                println!("set velocity: vx={vx} m/s, wz={wz} rad/s");
            }
            _ if line.starts_with("vel2 ") => {
                let mut parts = line.split_whitespace().skip(1);
                let left: i16 = parts.next().context("usage: vel2 <L> <R>")?.parse()?;
                let right: i16 = parts.next().context("usage: vel2 <L> <R>")?.parse()?;
                chassis.set_wheel_speeds_mm_s(left, right)?;
                println!("set wheel speeds: L={left} mm/s, R={right} mm/s");
            }
            other => println!("unknown command: {other}"),
        }
    }
    Ok(())
}
