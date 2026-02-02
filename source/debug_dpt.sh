#!/bin/bash
# Debug script for DPT target using ST-Link v3
# This script helps debug crashes by connecting GDB to the running firmware

set -e

BUILD_DIR="/Users/jm/Documents/Max 8/Library/oopsy_repo/source/build_reverb_dpt"
ELF_FILE="${BUILD_DIR}/build/reverb.elf"

if [ ! -f "$ELF_FILE" ]; then
    echo "Error: ELF file not found at $ELF_FILE"
    echo "Please build the project first using Oopsy in Max/MSP"
    exit 1
fi

echo "=========================================="
echo "DPT Debug Session with ST-Link v3"
echo "=========================================="
echo ""
echo "ELF file: $ELF_FILE"
echo ""

# Check if OpenOCD is already running
if nc -z localhost 3333 2>/dev/null; then
    echo "OpenOCD appears to be running on port 3333"
    echo "If you want to start a new session, kill the existing one first:"
    echo "  pkill openocd"
    echo ""
    read -p "Continue with existing OpenOCD? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
else
    echo "Starting OpenOCD in the background..."
    openocd -f interface/stlink.cfg -f target/stm32h7x.cfg -c init -c "reset init" > /tmp/openocd.log 2>&1 &
    OPENOCD_PID=$!
    echo "OpenOCD started (PID: $OPENOCD_PID)"
    echo "Waiting for OpenOCD to initialize..."
    sleep 2
    
    # Check if OpenOCD started successfully
    if ! nc -z localhost 3333 2>/dev/null; then
        echo "Error: OpenOCD failed to start. Check /tmp/openocd.log"
        kill $OPENOCD_PID 2>/dev/null || true
        exit 1
    fi
fi

echo ""
echo "Starting GDB..."
echo "=========================================="
echo ""
echo "Useful GDB commands:"
echo "  break main          - Set breakpoint at main()"
echo "  break DPT::Init      - Set breakpoint at DPT initialization"
echo "  break Dac7554::Init  - Set breakpoint at DAC7554 initialization"
echo "  continue            - Continue execution"
echo "  step                - Step into function"
echo "  next                - Step over function"
echo "  print variable      - Print variable value"
echo "  backtrace           - Show call stack"
echo "  info registers      - Show CPU registers"
echo "  monitor reset halt  - Reset and halt the MCU"
echo "  quit                - Exit GDB"
echo ""
echo "=========================================="
echo ""

# Start GDB
arm-none-eabi-gdb \
    -ex "target extended-remote localhost:3333" \
    -ex "monitor arm semihosting enable" \
    -ex "monitor reset halt" \
    -ex "load" \
    -ex "monitor reset init" \
    "$ELF_FILE"

echo ""
echo "Debug session ended."
echo "If you started OpenOCD with this script, you may want to kill it:"
echo "  pkill openocd"
