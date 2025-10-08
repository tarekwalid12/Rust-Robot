#!/bin/bash
# Quick flash script for RP2350

echo "🚀 Quick Flash for RP2350"
echo ""

# Build release
echo "1️⃣  Building release binary..."
cargo build --release || exit 1

PICOTOOL="$HOME/.pico-sdk/picotool/2.2.0/picotool/picotool"
BINARY="target/thumbv8m.main-none-eabihf/release/robot-embassy"
UF2="target/thumbv8m.main-none-eabihf/release/robot-embassy.uf2"

# Always use picotool to create UF2 with correct family ID
if [ -f "$PICOTOOL" ]; then
    # Check if device is in BOOTSEL mode
    if "$PICOTOOL" info -d 2>/dev/null | grep -q "RP2350"; then
        echo "2️⃣  Flashing ELF directly with picotool..."
        "$PICOTOOL" load "$BINARY" -t elf --family rp2350-arm-s --verify --execute
        echo ""
        echo "✅ DONE! Your robot should be running now."
        exit 0
    fi
    
    # No device found, create UF2 for manual drag-and-drop
    echo "2️⃣  Creating UF2 with picotool..."
    "$PICOTOOL" uf2 convert "$BINARY" "$UF2" --family rp2350-arm-s
    
    echo "3️⃣  Looking for RP2350 drive..."
    # Try common mount points
    for vol in "/Volumes/RP2350" "/Volumes/RPI-RP2"; do
        if [ -d "$vol" ]; then
            cp "$UF2" "$vol/"
            echo ""
            echo "✅ DONE! Copied RP2350-compatible UF2 to $vol"
            echo "   Device will reset automatically."
            exit 0
        fi
    done
    
    echo ""
    echo "⚠️  Created UF2: $UF2"
    echo "   Manually copy it to your RP2350 drive"
    exit 0
else
    echo ""
    echo "❌ picotool not found at $PICOTOOL"
    echo "   RP2350 requires picotool to create proper UF2 files"
    echo "   elf2uf2-rs doesn't support RP2350 yet"
    exit 1
fi
