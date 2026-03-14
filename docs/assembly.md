# Assembly Guide — Motcore v0.1

> **Note:** This guide covers the v0.1 proof of concept using LEGO Technic components. A v0.2 guide with 3D-printed parts is in development.

---

## Prerequisites

### Components Required

| Component | Quantity |
|-----------|----------|
| 28BYJ-48 Stepper Motor + ULN2003 driver | 1 |
| SG90 Micro Servo | 4 |
| Arduino Uno R3 | 2 |
| MCUFRIEND TFT Touchscreen 2.4" | 1 |
| Jumper wires | ~30 |
| Breadboard | 1 |
| LEGO Technic axles (5mm, various lengths) | 4+ |
| LEGO Technic beams and connectors | As needed |

### Tools Required

- Small Phillips screwdriver
- USB cable (Type-A to Type-B) × 2
- Computer with Arduino IDE or PlatformIO installed

---

## Step 1 — Mechanical Assembly

### 1.1 Central Motor Mount

1. Mount the 28BYJ-48 stepper motor at the center of the LEGO frame
2. Secure with LEGO beams to ensure the motor shaft is perpendicular to the base
3. Attach a friction hub to the motor shaft

### 1.2 Output Axes

1. Insert four 5mm LEGO Technic axles perpendicular to the central shaft
2. Position them at 90° intervals around the central axis
3. Ensure each axle can slide laterally (5–10mm of travel)

### 1.3 Friction Wheels

1. Attach one SG90 servo per output axis
2. Mount a rubber-coated friction wheel on each servo arm
3. Adjust servo horn position so the wheel makes clean contact with the central shaft when engaged

### 1.4 Frame

1. Build a rigid LEGO frame to hold all components in alignment
2. Verify all axes rotate freely when not engaged
3. Check that friction contact is clean and consistent

---

## Step 2 — Electronics

Refer to [Wiring Diagram](wiring.md) for full pinout details.

1. Connect the ULN2003 driver board to the central stepper motor
2. Connect all four SG90 servos to the master Arduino
3. Attach the MCUFRIEND TFT touchscreen to the master Arduino
4. Connect the two Arduino boards via serial (TX/RX)
5. Power both boards via USB

---

## Step 3 — Firmware Upload

1. Upload `software/controller/` to the **master Arduino** (touchscreen + servo control)
2. Upload `software/driver/` to the **receiver Arduino** (stepper motor control)

See [Firmware Guide](firmware.md) for details.

---

## Step 4 — Testing

1. Power on both Arduino boards
2. The touchscreen should initialize and display the control interface
3. Test each axis independently using the touchscreen buttons
4. Verify the central motor rotates continuously
5. Check that each servo correctly engages and disengages its friction wheel

---

## Troubleshooting

See [Troubleshooting Guide](troubleshooting.md) for common issues.

---

## Next Steps

Once the v0.1 prototype is working, v0.2 will replace the LEGO frame with parametric 3D-printed components. See [3D Models](3d-models.md) for the design in progress.
