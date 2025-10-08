#!/bin/bash
# Quick flash script - simplified version

echo "🚀 Quick Flash for RP2350"
echo ""

# Build release
echo "1️⃣  Building release binary..."
cargo build --release || exit 1

# Check for elf2uf2-rs
if ! command -v elf2uf2-rs &> /dev/null; then
    echo "Installing elf2uf2-rs..."
    cargo install elf2uf2-rs --locked
fi

# Convert to UF2
echo "2️⃣  Converting to UF2..."
BINARY="target/thumbv8m.main-none-eabihf/release/robot-embassy"
UF2="target/thumbv8m.main-none-eabihf/release/robot-embassy.uf2"
elf2uf2-rs "$BINARY" "$UF2"

# Flash with picotool
PICOTOOL="$HOME/.pico-sdk/picotool/2.2.0/picotool/picotool"

if [ -f "$PICOTOOL" ]; then
    echo "3️⃣  Flashing with picotool..."
    "$PICOTOOL" load "$UF2" --verify --execute
    echo ""
    echo "✅ DONE! Your robot should be running now."
else
    echo "3️⃣  Copying to RP2350 volume..."
    
    # Try common mount points
    for vol in "/Volumes/RP2350" "/Volumes/RPI-RP2"; do
        if [ -d "$vol" ]; then
            cp "$UF2" "$vol/"
            echo ""
            echo "✅ DONE! Copied to $vol"
            echo "   Device will reset automatically."
            exit 0
        fi
    done
    
    echo ""
    echo "❌ No RP2350 found!"
    echo "   1. Hold BOOTSEL button"
    echo "   2. Plug in USB"
    echo "   3. Run this script again"
    exit 1
fi
