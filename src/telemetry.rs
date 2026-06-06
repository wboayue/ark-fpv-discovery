//! Serial output layer: text logging and binary telemetry framing, plus the runtime output-mode
//! / diagnostic flags. The RTIC tasks in `main.rs` stay thin — they call these helpers and never
//! hand-roll a `String`/`write!`/`serial.write` or touch the wire codec directly.
//!
//! Two output modes, switched at runtime (CLAUDE.md "Telemetry output"):
//! - **Text** (boot default): human-readable lines via [`log_fmt`]; a terminal user needs no setup.
//! - **Binary**: postcard + COBS [`discovery-telemetry`](wire) frames via [`emit_frame`], decoded
//!   losslessly by the host scope.
//!
//! Encoding is deliberately split from the serial lock: `usb_irq` runs *inside* `serial.lock`, and
//! re-locking the same RTIC resource deadlocks — so it builds bytes with [`encode_frame`] and
//! writes the unlocked port via [`write_frame`], while the spawned tasks (which hold a `Mutex`
//! proxy) use the locking [`emit_frame`] wrapper.

use core::fmt::Write;
use core::sync::atomic::{AtomicBool, Ordering};

use discovery_telemetry as wire;
use heapless::String;
use rtic::Mutex;
use rtic_monotonics::Monotonic; // brings `Mono::now()` / `.ticks()` into scope
use stm32h7xx_hal::usb_hs::{UsbBus, USB2};
use usbd_serial::SerialPort;

use crate::fusion::FusedState;
use crate::sensors::{BaroSample, ImuSample, MagSample};
use crate::{Mono, config};

/// The emit helpers are generic over the RTIC shared-resource proxy for the USB serial port.
/// `Mutex` isn't object-safe (its `lock` is generic), so this is a bound, not a `dyn` alias:
/// `impl SerialMutex` reads as "any lockable handle to our `SerialPort`".
pub(crate) trait SerialMutex: Mutex<T = SerialPort<'static, UsbBus<USB2>>> {}
impl<M: Mutex<T = SerialPort<'static, UsbBus<USB2>>>> SerialMutex for M {}

// --- Runtime diagnostic mode --------------------------------------------------
// Toggled by the 'd' byte over USB serial. When on, sensor tasks emit verbose register-level
// dumps instead of concise readings. Lock-free so any task reads it without an RTIC resource lock.
static DIAG: AtomicBool = AtomicBool::new(false);

pub fn diag_enabled() -> bool {
    DIAG.load(Ordering::Relaxed)
}

/// Flip the diagnostic flag; returns the new state.
pub fn toggle_diag() -> bool {
    !DIAG.fetch_xor(true, Ordering::Relaxed)
}

// --- Telemetry output mode ----------------------------------------------------
// Selects the encoder at each emit site: text (the boot default) or binary frames. Toggled by the
// 'b'/'t' bytes over USB serial. Lock-free, mirroring DIAG.
static OUTPUT_BINARY: AtomicBool = AtomicBool::new(config::DEFAULT_OUTPUT_BINARY);

pub fn output_is_binary() -> bool {
    OUTPUT_BINARY.load(Ordering::Relaxed)
}

pub fn set_output_binary(on: bool) {
    OUTPUT_BINARY.store(on, Ordering::Relaxed);
}

/// Short firmware commit (from `build.rs` `GIT_HASH`) as the NUL-padded `[u8; 8]` the telemetry
/// `Hello` frame carries. `GIT_HASH` is already capped to 7 chars, so it always fits.
pub fn fw_git() -> [u8; 8] {
    let mut out = [0u8; 8];
    let bytes = env!("GIT_HASH").as_bytes();
    let n = bytes.len().min(out.len());
    out[..n].copy_from_slice(&bytes[..n]);
    out
}

// --- Text logging -------------------------------------------------------------

/// Longest formatted log line we emit (the verbose mag diagnostic dump); sizes the buffer.
const LOG_LINE_CAP: usize = 192;

/// Lock the shared USB serial port and write `bytes`. Generic over the RTIC resource proxy so
/// every task shares one code path. Best-effort: write errors are dropped.
pub fn write_serial(serial: &mut impl SerialMutex, bytes: &[u8]) {
    serial.lock(|serial| {
        let _ = serial.write(bytes);
    });
}

/// Format and write one line to the shared serial port. Collapses the repeated
/// `String::new()` / `write!` / `write_serial` boilerplate, owning the one buffer size so call
/// sites carry no magic number. Best-effort: a line longer than `LOG_LINE_CAP` is truncated and
/// write errors are dropped. Pass the message with `format_args!`, e.g.
/// `log_fmt(&mut cx.shared.serial, format_args!("tick {}", n))`.
pub fn log_fmt(serial: &mut impl SerialMutex, args: core::fmt::Arguments) {
    let mut msg: String<LOG_LINE_CAP> = String::new();
    let _ = msg.write_fmt(args);
    write_serial(serial, msg.as_bytes());
}

// --- Binary telemetry framing -------------------------------------------------

/// Stamp `t_ms` from the monotonic, build a `Frame`, and COBS-encode it into `buf`. No lock, no
/// serial. The one place owning the encode buffer (`codec::MAX_FRAME`), as `log_fmt` owns
/// `LOG_LINE_CAP`. Returns `None` only if encoding overflows `buf`. The `Err` is dropped without
/// naming it, so no direct `postcard`/`serde` dependency leaks in.
pub fn encode_frame(msg: wire::Msg, buf: &mut [u8; wire::codec::MAX_FRAME]) -> Option<&[u8]> {
    // Mono is 1 kHz → ticks are ms; `ticks()` is u32 here, matching Frame.t_ms.
    let frame = wire::Frame { t_ms: Mono::now().ticks(), msg };
    wire::codec::encode(&frame, buf).ok().map(|w| &*w)
}

/// Encode `msg` and write it to an already-unlocked serial port (the `usb_irq` path).
pub fn write_frame(serial: &mut SerialPort<'static, UsbBus<USB2>>, msg: wire::Msg) {
    let mut buf = [0u8; wire::codec::MAX_FRAME];
    if let Some(w) = encode_frame(msg, &mut buf) {
        let _ = serial.write(w);
    }
}

/// Lock the shared serial port and emit one binary frame (the spawned-task path).
pub fn emit_frame(serial: &mut impl SerialMutex, msg: wire::Msg) {
    serial.lock(|s| write_frame(s, msg));
}

/// Format `args` into a `Status` frame's NUL-padded text (silently truncated at 48 bytes — ASCII
/// status text). The binary equivalent of a text status line.
pub fn status_msg(level: wire::Level, args: core::fmt::Arguments) -> wire::Msg {
    let mut s: String<48> = String::new();
    let _ = s.write_fmt(args);
    let mut text = [0u8; 48];
    let bytes = s.as_bytes();
    let n = bytes.len().min(text.len());
    text[..n].copy_from_slice(&bytes[..n]);
    wire::Msg::Status(wire::Status { level, text })
}

/// Emit a non-data status/notice line in whichever mode is active: a `Status` frame in binary
/// mode, an `\r\n`-terminated text line in text mode. For lines whose human text is identical in
/// both modes (sensor bring-up identity, errors). Pass `args` without a trailing newline.
pub fn emit_line(serial: &mut impl SerialMutex, level: wire::Level, args: core::fmt::Arguments) {
    if output_is_binary() {
        emit_frame(serial, status_msg(level, args));
    } else {
        log_fmt(serial, format_args!("{args}\r\n"));
    }
}

// --- Per-sample rendering -----------------------------------------------------
// One place that maps each sensor/fused sample to its `Msg` frame (binary) or its concise text
// line, so the RTIC tasks just call `log_*` and stay thin. Verbose diagnostic dumps (which read
// live driver registers / estimator internals) stay in the tasks — they're text-only and need
// more than the sample.

/// Emit one IMU sample: an `Imu` frame in binary mode, the `imu[n] …` text line otherwise. On the
/// first logged sample (`n == IMU_LOG_DIV`) binary mode also emits a one-shot `Status` carrying the
/// constant WHO_AM_I `id` (the data frame omits it). `exp` is the expected WHO_AM_I.
pub fn log_imu(serial: &mut impl SerialMutex, n: u32, id: u8, exp: u8, s: &ImuSample) {
    if output_is_binary() {
        if n == config::IMU_LOG_DIV {
            emit_frame(serial, status_msg(wire::Level::Info, format_args!("imu id=0x{id:02x} (exp {exp:02x})")));
        }
        emit_frame(
            serial,
            wire::Msg::Imu(wire::Imu { accel_g: s.accel_g, gyro_dps: s.gyro_dps, temp_c: s.temp_c }),
        );
    } else {
        log_fmt(
            serial,
            format_args!(
                "imu[{}] id=0x{:02x}(exp {:02x}) accel[g]={:.2},{:.2},{:.2} gyro[dps]={:.1},{:.1},{:.1} temp={:.1}C\r\n",
                n, id, exp,
                s.accel_g[0], s.accel_g[1], s.accel_g[2],
                s.gyro_dps[0], s.gyro_dps[1], s.gyro_dps[2],
                s.temp_c,
            ),
        );
    }
}

/// Emit one barometer sample: a `Baro` frame in binary mode, the concise `baro …` text otherwise.
pub fn log_baro(serial: &mut impl SerialMutex, s: &BaroSample) {
    if output_is_binary() {
        emit_frame(serial, wire::Msg::Baro(wire::Baro { pressure_hpa: s.pressure_hpa, temp_c: s.temp_c }));
    } else {
        log_fmt(serial, format_args!("baro press={:.2}hPa temp={:.2}C\r\n", s.pressure_hpa, s.temp_c));
    }
}

/// Emit one magnetometer sample: a `Mag` frame in binary mode, the concise `mag …` text otherwise.
/// (The verbose register dump stays in the task — it reads live config registers.)
pub fn log_mag(serial: &mut impl SerialMutex, s: &MagSample) {
    if output_is_binary() {
        emit_frame(serial, wire::Msg::Mag(wire::Mag { field_ut: s.field_ut, temp_c: s.temp_c }));
    } else {
        log_fmt(
            serial,
            format_args!(
                "mag field[uT]={:.1},{:.1},{:.1} temp={:.1}C\r\n",
                s.field_ut[0], s.field_ut[1], s.field_ut[2], s.temp_c,
            ),
        );
    }
}

/// Emit one fused state: a `Fused` frame in binary mode, the concise `fus …` text otherwise.
/// (The verbose vertical-channel dump stays in the task — it reads estimator internals.) Note the
/// field renames onto the wire type: `vertical_velocity → vertical_speed_mps`,
/// `baro_residual → baro_residual_m`.
pub fn log_fused(serial: &mut impl SerialMutex, s: &FusedState) {
    if output_is_binary() {
        emit_frame(
            serial,
            wire::Msg::Fused(wire::Fused {
                roll_deg: s.roll_deg,
                pitch_deg: s.pitch_deg,
                yaw_deg: s.yaw_deg,
                altitude_m: s.altitude_m,
                vertical_speed_mps: s.vertical_velocity,
                baro_residual_m: s.baro_residual,
            }),
        );
    } else {
        log_fmt(
            serial,
            format_args!(
                "fus roll={:.1} pitch={:.1} yaw={:.1}deg alt={:.2}m vz={:.2}m/s\r\n",
                s.roll_deg, s.pitch_deg, s.yaw_deg, s.altitude_m, s.vertical_velocity,
            ),
        );
    }
}
