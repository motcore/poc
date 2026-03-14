# Firmware Guide — Motcore v0.1

---

## Architecture

The control system uses two Arduino Uno boards communicating over serial:

```
┌─────────────────────────────┐     Serial      ┌─────────────────────────┐
│     Master Arduino          │ ──────────────► │    Receiver Arduino     │
│                             │                  │                         │
│  - TFT Touchscreen UI       │                  │  - Stepper motor control│
│  - Servo control (4 axes)   │                  │  - Receives commands    │
│  - User input processing    │                  │    from master          │
└─────────────────────────────┘                  └─────────────────────────┘
```

---

## Repository Structure

```
software/
├── controller/          # Master Arduino firmware
│   ├── controller.ino   # Main file
│   └── ...
└── driver/              # Receiver Arduino firmware
    ├── driver.ino        # Main file
    └── ...
```

---

## Dependencies

### PlatformIO (recommended)

Dependencies are declared in `platformio.ini`. Run:

```bash
pio lib install
```

### Arduino IDE

Install the following libraries via Library Manager:

| Library | Purpose |
|---|---|
| `MCUFRIEND_kbv` | TFT display driver |
| `Adafruit GFX` | Graphics primitives |
| `TouchScreen` | Touchscreen input |
| `Servo` | Servo motor control |
| `AccelStepper` | Stepper motor control |

---

## Upload Instructions

> **Important:** Disconnect the TX/RX serial wires between the two Arduinos before uploading, then reconnect after.

### 1. Upload to Master Arduino

```bash
# PlatformIO
cd software/controller
pio run --target upload

# Arduino IDE
# Open software/controller/controller.ino
# Select board: Arduino Uno
# Select port: COM port of master Arduino
# Click Upload
```

### 2. Upload to Receiver Arduino

```bash
# PlatformIO
cd software/driver
pio run --target upload

# Arduino IDE
# Open software/driver/driver.ino
# Select board: Arduino Uno
# Select port: COM port of receiver Arduino
# Click Upload
```

---

## Serial Protocol

The master sends single-byte commands to the receiver over serial at **9600 baud**.

| Command | Action |
|---|---|
| `'F'` | Stepper forward |
| `'B'` | Stepper backward |
| `'S'` | Stepper stop |

---

## Servo Positions

Each servo has three positions:

| State | Angle | Description |
|---|---|---|
| Neutral | 90° | No contact with central shaft |
| Engage CW | 0° | Friction contact — clockwise rotation |
| Engage CCW | 180° | Friction contact — counterclockwise rotation |

These values can be tuned in `controller.ino` based on your physical assembly.

---

## Customization

### Changing servo angles

In `controller.ino`, adjust the `SERVO_NEUTRAL`, `SERVO_CW`, `SERVO_CCW` constants to match your mechanical setup.

### Adding more axes

1. Connect additional SG90 servos to available PWM pins
2. Add servo objects and pin definitions in `controller.ino`
3. Add corresponding UI buttons to the touchscreen layout

### Changing stepper speed

In `driver.ino`, adjust the `STEPPER_SPEED` constant (steps per second).
