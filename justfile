# ark-discovery — build & flash helpers. Run `just` to list.

flash_addr := "0x08000000"
bin        := "firmware.bin"

# List recipes (default).
default:
    @just --list

# Release build (use this for flashing).
build:
    cargo build --release

# Debug build.
build-debug:
    cargo build

# Build + objcopy to a raw binary for DFU.
bin: build
    cargo objcopy --release -- -O binary {{bin}}

# Requires firmware that handles 'r'; if the board is hung, use BOOT0+RESET instead.
# Reboot the running firmware into the ROM bootloader and wait for the DFU device.
dfu:
    #!/usr/bin/env bash
    set -euo pipefail
    tty=$(ls /dev/cu.usbmodem* 2>/dev/null | head -1)
    if [ -z "$tty" ]; then echo "no CDC port found"; exit 1; fi
    echo "rebooting $tty into DFU..."
    printf 'r' > "$tty"
    end=$((SECONDS+15))
    while [ $SECONDS -lt $end ]; do
        if dfu-util -l 2>/dev/null | grep -q "0483:df11"; then
            echo "DFU up."; exit 0
        fi
    done
    echo "no DFU device after 15s — try BOOT0+RESET"; exit 1

# Flash over DFU. Board must already be in DFU (run `just dfu` first, or BOOT0+RESET).
flash: bin
    #!/usr/bin/env bash
    set -euo pipefail
    if ! dfu-util -l 2>/dev/null | grep -q "0483:df11"; then
        echo "no DFU device (0483:df11). Run 'just dfu' or BOOT0+RESET first."; exit 1
    fi
    dfu-util -a 0 -s {{flash_addr}}:leave -D {{bin}} || true
    echo
    echo ">>> Flash done. PRESS NRST to boot cleanly (:leave auto-run is unreliable)."
    echo ">>> 'get_status' / 'Invalid DFU suffix' warnings above are benign."

# Read the serial port (Ctrl-C to stop). Expect 'tick N' once/sec.
monitor:
    #!/usr/bin/env bash
    set -euo pipefail
    tty=$(ls /dev/cu.usbmodem* 2>/dev/null | head -1)
    if [ -z "$tty" ]; then echo "no CDC port found"; exit 1; fi
    echo "reading $tty (Ctrl-C to stop)"
    stty -f "$tty" 115200 raw -echo
    cat "$tty"

# Send 'r' to reboot the running firmware into the ROM bootloader (no wait).
reboot:
    #!/usr/bin/env bash
    set -euo pipefail
    tty=$(ls /dev/cu.usbmodem* 2>/dev/null | head -1)
    if [ -z "$tty" ]; then echo "no CDC port found"; exit 1; fi
    printf 'r' > "$tty"
    echo "sent 'r' to $tty"

# Show DFU devices (looking for 0483:df11).
dfu-list:
    dfu-util -l

# Remove build artifacts.
clean:
    cargo clean
    rm -f {{bin}}
