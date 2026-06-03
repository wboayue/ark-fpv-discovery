//! Injects the short git commit into the build as `GIT_HASH` for the telemetry `Hello` frame
//! (`fw_git`). Best-effort: a missing/failed `git` (sandbox, tarball, no-repo checkout) falls
//! back to "unknown" rather than failing the build. Re-runs when HEAD moves so the hash stays
//! fresh.

use std::process::Command;

fn main() {
    let hash = Command::new("git")
        .args(["rev-parse", "--short=7", "HEAD"])
        .output()
        .ok()
        .filter(|o| o.status.success())
        .and_then(|o| String::from_utf8(o.stdout).ok())
        .map(|s| s.trim().to_string())
        // A short hash can auto-extend past 7 chars when 7 would be ambiguous; the Hello.fw_git
        // field is [u8; 8] (7 chars + NUL), so cap it.
        .map(|s| s.chars().take(7).collect::<String>())
        .filter(|s| !s.is_empty())
        .unwrap_or_else(|| "unknown".to_string());

    println!("cargo:rustc-env=GIT_HASH={hash}");
    // Rebuild when the checked-out commit changes, else GIT_HASH goes stale.
    println!("cargo:rerun-if-changed=.git/HEAD");
    println!("cargo:rerun-if-changed=.git/refs");
}
