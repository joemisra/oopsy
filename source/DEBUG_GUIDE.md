# Debugging DPT Crashes with ST-Link v3

## Quick Start

1. **Make sure your ST-Link v3 is connected** to the DPT board

2. **Run the debug script:**
   ```bash
   cd "/Users/jm/Documents/Max 8/Library/oopsy_repo/source"
   ./debug_dpt.sh
   ```

3. **In GDB, set breakpoints at key initialization points:**
   ```gdb
   break main
   break daisy::dpt::DPT::Init
   break Dac7554::Init
   break daisy::dpt::DPT::InitMidi
   continue
   ```

4. **When it crashes, check the backtrace:**
   ```gdb
   backtrace
   info registers
   ```

## Common Crash Points

Based on the code, likely crash points are:

1. **DAC7554 Initialization** (`Dac7554::Init()`)
   - SPI initialization might fail if hardware isn't connected
   - Check if SPI2 pins are configured correctly
   - Pins: D1 (CS), D8 (MISO), D9 (MOSI), D10 (SCLK)

2. **MIDI Initialization** (`DPT::InitMidi()`)
   - UART initialization on pins A8/A9
   - Check if MIDI hardware is connected

3. **Audio Initialization** (`AudioHandle::Init()`)
   - SAI peripheral initialization
   - Codec initialization via I2C

## Manual Debugging Steps

### Option 1: Use the debug script (recommended)
```bash
./debug_dpt.sh
```

### Option 2: Manual OpenOCD + GDB

**Terminal 1 - Start OpenOCD:**
```bash
openocd -f interface/stlink.cfg -f target/stm32h7x.cfg
```

**Terminal 2 - Start GDB:**
```bash
cd "/Users/jm/Documents/Max 8/Library/oopsy_repo/source/build_squine_dpt"
arm-none-eabi-gdb build/squine.elf
```

Then in GDB:
```gdb
target extended-remote localhost:3333
monitor reset halt
load
monitor reset init
break main
continue
```

## Checking Where It Crashes

1. **Set breakpoint at main:**
   ```gdb
   break main
   continue
   ```

2. **Step through initialization:**
   ```gdb
   step    # Step into functions
   next    # Step over functions
   ```

3. **When it crashes, get the backtrace:**
   ```gdb
   backtrace
   frame 0    # Look at the current frame
   print variable_name  # Print variable values
   ```

## Potential Issues

### Issue 1: DAC7554 Hardware Not Connected
If the DAC7554 expander isn't connected, SPI initialization might hang or crash.

**Solution:** Comment out `dac_exp.Init()` temporarily to test if this is the issue.

### Issue 2: MIDI Hardware Conflict
MIDI uses pins A8/A9. If there's a conflict, initialization might fail.

**Solution:** Check if `ENABLE_MIDI` is set correctly in `daisy_dpt.h`.

### Issue 3: Stack Overflow
If the stack is too small, the program might crash during initialization.

**Solution:** Check linker script for stack size.

## Serial Output (Alternative)

If GDB isn't available, you can add serial output to see where it crashes:

1. Connect to the UART (usually on USB or a serial port)
2. Add `printf` statements in the code
3. Monitor the serial output

## VS Code Debugging

If you prefer VS Code:

1. Install the "Cortex-Debug" extension
2. Create `.vscode/launch.json`:
```json
{
  "configurations": [
    {
      "name": "Debug DPT",
      "type": "cortex-debug",
      "request": "launch",
      "servertype": "openocd",
      "cwd": "${workspaceFolder}",
      "executable": "${workspaceFolder}/source/build_squine_dpt/build/squine.elf",
      "configFiles": [
        "interface/stlink.cfg",
        "target/stm32h7x.cfg"
      ],
      "runToEntryPoint": "main"
    }
  ]
}
```

## Next Steps

Once you identify where it crashes:
1. Check the hardware connections
2. Verify pin configurations
3. Check if external hardware (DAC7554, MIDI) is connected
4. Review the initialization sequence
