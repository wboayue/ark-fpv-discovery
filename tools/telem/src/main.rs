//! `telem` — host-side reader/decoder for the ark-fpv-discovery binary telemetry stream.
//!
//! Opens the board's USB CDC serial port, sends `b` to put the firmware in binary mode, then
//! COBS-deframes + postcard-decodes each [`discovery_telemetry::Frame`] and prints it as a
//! readable line. It decodes with the **same crate the firmware encodes with**
//! (`discovery-telemetry`), so the wire format can never drift between the two sides.
//!
//! This is a `std` host tool and is deliberately **not** part of the firmware build — it lives in
//! its own workspace member (`tools/telem`); see the workspace note in the root `Cargo.toml`.
//!
//! Quit with Ctrl-C; closing the port drops DTR, which the firmware watches and uses to revert to
//! text mode, so the next terminal user gets readable lines without sending `t`.

use anyhow::{Context, Result};
use clap::Parser;
use discovery_telemetry::{Board, Frame, Level, Msg, PROTOCOL_VERSION, codec::Decoder};
use serialport::SerialPortType;
use std::io::{Read, Write};
use std::time::Duration;

/// Read and decode the ark-fpv-discovery binary telemetry stream.
#[derive(Parser)]
#[command(version, about)]
struct Args {
    /// Serial port (default: auto-detect the first USB serial port,
    /// e.g. /dev/cu.usbmodem* on macOS, /dev/ttyACM* on Linux).
    port: Option<String>,

    /// Baud rate. USB CDC ignores it, but the OS serial API still requires a value.
    #[arg(long, default_value_t = 115_200)]
    baud: u32,

    /// Don't send `b` — assume the firmware is already streaming binary.
    #[arg(long)]
    no_switch: bool,
}

/// First USB serial port, falling back to whatever port exists.
fn detect_port() -> Result<String> {
    let ports = serialport::available_ports().context("listing serial ports")?;
    ports
        .iter()
        .find(|p| matches!(p.port_type, SerialPortType::UsbPort(_)))
        .or_else(|| ports.first())
        .map(|p| p.port_name.clone())
        .context("no serial ports found — pass one explicitly")
}

fn main() -> Result<()> {
    let args = Args::parse();
    let port_name = match args.port {
        Some(p) => p,
        None => detect_port()?,
    };

    eprintln!("telem: opening {port_name} @ {} baud", args.baud);
    let mut port = serialport::new(&port_name, args.baud)
        .timeout(Duration::from_millis(200))
        .open()
        .with_context(|| format!("opening {port_name}"))?;

    if !args.no_switch {
        port.write_all(b"b")
            .context("sending 'b' (switch to binary)")?;
        port.flush().ok();
        eprintln!(
            "telem: sent 'b' — decoding frames (Ctrl-C to quit; firmware reverts to text on close)"
        );
    }

    let mut dec = Decoder::new();
    let mut buf = [0u8; 2048];
    loop {
        match port.read(&mut buf) {
            Ok(0) => {}
            Ok(n) => {
                let dropped = dec.push(&buf[..n], |f| print_frame(&f));
                if dropped > 0 {
                    // Expected once or twice right after `b` (stale text flushed); persistent
                    // drops mean corruption or a protocol-version skew.
                    eprintln!("telem: dropped {dropped} frame(s) (resync / version skew)");
                }
            }
            // A read timeout just means no bytes this interval — keep waiting.
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {}
            Err(e) => return Err(e).context("reading serial"),
        }
    }
}

/// NUL-trim a fixed-size ASCII field (`fw_git`, `Status::text`) into a string.
fn nul_trimmed(b: &[u8]) -> std::borrow::Cow<'_, str> {
    let end = b.iter().position(|&c| c == 0).unwrap_or(b.len());
    String::from_utf8_lossy(&b[..end])
}

fn print_frame(f: &Frame) {
    let t = f.t_ms;
    match f.msg {
        Msg::Hello(h) => {
            let board = match h.board {
                Board::ArkDiscovery => "ArkDiscovery",
                Board::HolybroDiscovery => "HolybroDiscovery",
                Board::Unknown => "Unknown",
            };
            let skew = if h.proto != PROTOCOL_VERSION {
                format!("  !! proto mismatch (host expects {PROTOCOL_VERSION})")
            } else {
                String::new()
            };
            println!(
                "[{t:>8}] HELLO proto={} board={board} fw={}{skew}",
                h.proto,
                nul_trimmed(&h.fw_git)
            );
        }
        Msg::Tick(n) => println!("[{t:>8}] tick {n}"),
        Msg::Imu(s) => println!(
            "[{t:>8}] imu accel[g]={:.2},{:.2},{:.2} gyro[dps]={:.1},{:.1},{:.1} temp={:.1}C",
            s.accel_g[0],
            s.accel_g[1],
            s.accel_g[2],
            s.gyro_dps[0],
            s.gyro_dps[1],
            s.gyro_dps[2],
            s.temp_c
        ),
        Msg::Baro(s) => {
            println!(
                "[{t:>8}] baro press={:.2}hPa temp={:.2}C",
                s.pressure_hpa, s.temp_c
            )
        }
        Msg::Mag(s) => println!(
            "[{t:>8}] mag field[uT]={:.1},{:.1},{:.1} temp={:.1}C",
            s.field_ut[0], s.field_ut[1], s.field_ut[2], s.temp_c
        ),
        Msg::Fused(s) => println!(
            "[{t:>8}] fus roll={:.1} pitch={:.1} yaw={:.1}deg alt={:.2}m vz={:.2}m/s resid={:.2}m",
            s.roll_deg,
            s.pitch_deg,
            s.yaw_deg,
            s.altitude_m,
            s.vertical_speed_mps,
            s.baro_residual_m
        ),
        Msg::Status(s) => {
            let lvl = match s.level {
                Level::Info => "INFO",
                Level::Warn => "WARN",
                Level::Error => "ERROR",
            };
            println!("[{t:>8}] [{lvl}] {}", nul_trimmed(&s.text));
        }
    }
}
