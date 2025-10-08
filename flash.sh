#!/bin/bash
# Flash script for RP2350 in BOOTSEL mode using picotool

set -e

echo "🔨 Building release binary..."
cargo build --release

echo "📦 Converting ELF to UF2..."
BINARY="target/thumbv8m.main-none-eabihf/release/robot-embassy"
UF2_FILE="target/thumbv8m.main-none-eabihf/release/robot-embassy.uf2"

# Use elf2uf2-rs to convert (install with: cargo install elf2uf2-rs)
if command -v elf2uf2-rs &> /dev/null; then
    elf2uf2-rs "$BINARY" "$UF2_FILE"
    echo "✅ UF2 created: $UF2_FILE"
else
    echo "⚠️  elf2uf2-rs not found. Installing..."
    cargo install elf2uf2-rs --locked
    elf2uf2-rs "$BINARY" "$UF2_FILE"
fi

echo "🔍 Looking for RP2350 in BOOTSEL mode..."

# Check if picotool is available
PICOTOOL="${HOME}/.pico-sdk/picotool/2.2.0/picotool/picotool"

if [ -f "$PICOTOOL" ]; then
    echo "📡 Using picotool to flash..."
    
    # Load the UF2 file and reset
    "$PICOTOOL" load "$UF2_FILE" --verify --execute
    
    echo "✅ Flashed successfully! Device should be running."
else
    echo "⚠️  picotool not found at $PICOTOOL"
    echo ""
    echo "Alternative: Copy UF2 file manually"
    echo "  1. Hold BOOTSEL button and plug in RP2350"
    echo "  2. Copy: $UF2_FILE"
    echo "  3. To: /Volumes/RP2350 (or RP2040)"
    echo ""
    
    # Try to detect mounted volume and copy automatically
    if [ -d "/Volumes/RP2350" ]; then
        echo "📋 Detected /Volumes/RP2350, copying UF2..."
        cp "$UF2_FILE" "/Volumes/RP2350/"
        echo "✅ Copied! Waiting for device to reset..."
        sleep 2
        echo "✅ Done!"
    elif [ -d "/Volumes/RPI-RP2" ]; then
        echo "📋 Detected /Volumes/RPI-RP2, copying UF2..."
        cp "$UF2_FILE" "/Volumes/RPI-RP2/"
        echo "✅ Copied! Waiting for device to reset..."
        sleep 2
        echo "✅ Done!"
    else
        echo "❌ No RP2350 volume detected. Please mount it first."
        exit 1
    fi
fi
