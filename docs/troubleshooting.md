# Troubleshooting Guide — Motcore v0.1

---

## Mechanical Issues

### Axis doesn't move when servo engages

**Symptoms:** Servo moves to engaged position but output axis stays still.

**Possible causes:**
- Friction wheel not making contact with the central shaft
- Central motor not spinning
- Friction surface too smooth

**Solutions:**
1. Check that the central stepper motor is running (listen for the motor sound)
2. Adjust the servo angle constants in `controller.ino` to increase contact pressure
3. Add a rubber band or O-ring to the friction wheel for better grip
4. Verify the output axis slides freely and is aligned with the friction wheel

---

### Axis moves erratically or inconsistently

**Symptoms:** Axis moves sometimes but not reliably.

**Possible causes:**
- Insufficient friction contact pressure
- Loose mechanical assembly
- Servo positioning inconsistency

**Solutions:**
1. Increase contact pressure by adjusting servo engaged angle
2. Check all LEGO connections are tight
3. Ensure friction wheel surface is clean (no grease or dust)

---

### All axes move simultaneously

**Symptoms:** Multiple axes move when only one should.

**Possible causes:**
- Friction wheels from different axes are touching

**Solutions:**
1. Increase spacing between output axes
2. Check that only the target servo is engaging

---

## Electronics Issues

### Touchscreen doesn't initialize

**Symptoms:** Black screen or garbled display on startup.

**Possible causes:**
- MCUFRIEND shield not fully seated
- Wrong display driver ID

**Solutions:**
1. Remove and reseat the TFT shield firmly
2. Run the `LCD_ID_readreg` example from MCUFRIEND_kbv library to identify your display ID
3. Verify the correct driver is selected in `controller.ino`

---

### Servos jitter or don't reach position

**Symptoms:** Servos vibrate continuously or don't reach the commanded angle.

**Possible causes:**
- Insufficient power supply
- PWM pin conflict with other peripherals

**Solutions:**
1. Power servos directly from a 5V rail, not from the Arduino 5V pin
2. Check that servo signal wires are connected to PWM-capable pins (marked `~` on Arduino Uno)
3. Add a 100µF capacitor across the servo power rail

---

### Stepper motor doesn't spin

**Symptoms:** No movement from the central 28BYJ-48 motor.

**Possible causes:**
- Loose connection to ULN2003 driver
- Receiver Arduino not receiving serial commands

**Solutions:**
1. Check all four wires between ULN2003 and receiver Arduino
2. Open Serial Monitor on the master Arduino and verify commands are being sent
3. Verify TX/RX serial wires are correctly connected (TX→RX, RX→TX) and GND is shared
4. Disconnect serial wires, upload fresh firmware to receiver Arduino, reconnect

---

### Serial communication not working

**Symptoms:** Stepper doesn't respond to touchscreen input.

**Solutions:**
1. Ensure GND is shared between both Arduinos
2. Verify baud rate is 9600 in both `controller.ino` and `driver.ino`
3. Swap TX/RX wires (common mistake)
4. Remember to **disconnect serial wires before uploading** firmware

---

## Software Issues

### Compilation errors

**Solutions:**
1. Ensure all required libraries are installed (see [Firmware Guide](firmware.md))
2. Select the correct board: **Arduino Uno**
3. Check PlatformIO or Arduino IDE version compatibility

---

## Still stuck?

Open an issue on [GitHub Issues](../../issues) with:
- Description of the problem
- What you've already tried
- Photos of your assembly if it's a mechanical issue
- Serial Monitor output if it's an electronics/software issue
