#!/usr/bin/env bash
# build_all.sh — Build CSI Radar firmware for all device targets
#
# Prerequisites:
#   . ~/esp/esp-idf/export.sh        (sets IDF_PATH, adds idf.py to PATH)
#
# Usage:
#   chmod +x build_all.sh
#   ./build_all.sh                   (build everything)
#   ./build_all.sh tx-esp32          (build one variant)
#
# Each variant gets its own build directory so sdkconfigs don't collide.
# First-time build generates sdkconfig from defaults.
# Customize device-specific settings (SSID, DEVICE_ID, role) with:
#   idf.py -B build/<variant> menuconfig
#
# Flash a variant:
#   idf.py -B build/<variant> -p /dev/ttyUSB0 flash monitor

set -euo pipefail

if [[ -z "${IDF_PATH:-}" ]]; then
    echo "ERROR: IDF_PATH not set. Run: . ~/esp/esp-idf/export.sh"
    exit 1
fi

PROJECT_ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "$PROJECT_ROOT"

# ── Variant table: name  target     role   device_id_hint ────────────────────
declare -A VARIANTS=(
    ["tx-esp32"]="esp32"
    ["tx-esp32s3"]="esp32s3"
    ["tx-esp32c5"]="esp32c5"
    ["rx-esp32s3"]="esp32s3"
)

ROLE_HINT=(
    ["tx-esp32"]="TX  → WROOM x2"
    ["tx-esp32s3"]="TX  → Atom S3, AtomFly"
    ["tx-esp32c5"]="TX  → Waveshare C5, T-Dongle C5 (5GHz capable)"
    ["rx-esp32s3"]="RX  → XIAO ESP32-S3 Sense, Cardputer ADV"
)

build_variant() {
    local name="$1"
    local target="${VARIANTS[$name]}"
    local build_dir="build/${name}"

    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "  Building: ${name}"
    echo "  Target:   ${target}"
    echo "  Role:     ${ROLE_HINT[$name]}"
    echo "  Dir:      ${build_dir}"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

    mkdir -p "$build_dir"

    idf.py \
        -D IDF_TARGET="${target}" \
        -B "${build_dir}" \
        build

    local bin="${build_dir}/csi_radar.bin"
    if [[ -f "$bin" ]]; then
        echo "  ✓ ${bin}  ($(du -sh "$bin" | cut -f1))"
        echo "  Flash: idf.py -B ${build_dir} -p /dev/ttyUSB0 flash"
    else
        echo "  ✗ Build succeeded but binary not found?"
    fi
}

# ── Main ─────────────────────────────────────────────────────────────────────
if [[ $# -gt 0 ]]; then
    # Build specific variant(s) passed as arguments
    for variant in "$@"; do
        if [[ -z "${VARIANTS[$variant]+x}" ]]; then
            echo "Unknown variant: $variant"
            echo "Available: ${!VARIANTS[*]}"
            exit 1
        fi
        build_variant "$variant"
    done
else
    # Build everything
    echo "Building all variants..."
    for variant in "tx-esp32" "tx-esp32s3" "tx-esp32c5" "rx-esp32s3"; do
        build_variant "$variant"
    done
fi

echo ""
echo "═══ Done. Before first flash, customize each build: ════════════════════"
echo "  idf.py -B build/tx-esp32    menuconfig   # set DEVICE_ID, channel"
echo "  idf.py -B build/tx-esp32s3  menuconfig   # set DEVICE_ID, channel"
echo "  idf.py -B build/tx-esp32c5  menuconfig   # set DEVICE_ID, channel (36+ for 5GHz)"
echo "  idf.py -B build/rx-esp32s3  menuconfig   # set DEVICE_ID, SSID, password, notify URL"
echo "════════════════════════════════════════════════════════════════════════"
