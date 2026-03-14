# Wiring Diagram — Motcore v0.1

---

## System Overview

```
[TFT Touchscreen] ──── [Master Arduino Uno]
                              │
                    ┌─────────┴──────────┐
                    │   Serial TX/RX     │
                    │                    │
              [4x SG90 Servos]   [Receiver Arduino Uno]
                                         │
                                  [ULN2003 Driver]
                                         │
                                 [28BYJ-48 Stepper]
```

---

## Master Arduino — Pinout

### TFT Touchscreen (MCUFRIEND 2.4")

The MCUFRIEND shield plugs directly onto the Master Arduino Uno headers.

| TFT Function | Arduino Pin |
|---|---|
| LCD data bus | D2–D9 |
| LCD RS | A0 |
| LCD WR | A1 |
| LCD CS | A2 |
| LCD RST | A3 |
| Touch XP | A2 |
| Touch YP | A3 |
| Touch XM | D8 |
| Touch YM | D9 |

### Servo Motors

| Servo | Arduino Pin |
|---|---|
| Servo Axis 1 | D3 |
| Servo Axis 2 | D5 |
| Servo Axis 3 | D6 |
| Servo Axis 4 | D9 |

> **Note:** Servo power (5V/GND) should be taken directly from the USB power rail, not from the Arduino 5V pin, to avoid brownouts.

### Serial Communication (to Receiver Arduino)

| Signal | Master Pin | Receiver Pin |
|---|---|---|
| TX → RX | D1 (TX) | D0 (RX) |
| RX ← TX | D0 (RX) | D1 (TX) |
| GND | GND | GND |

> **Important:** Disconnect the serial wires before uploading firmware to either Arduino.

---

## Receiver Arduino — Pinout

### ULN2003 Stepper Driver

| ULN2003 Input | Arduino Pin |
|---|---|
| IN1 | D8 |
| IN2 | D9 |
| IN3 | D10 |
| IN4 | D11 |

### Power

| Connection | Detail |
|---|---|
| Motor power | 5V via USB |
| GND | Shared with Master Arduino |

---

## Power Notes

- Both Arduinos are powered independently via USB (5V)
- Share a common GND between both boards
- Total current draw: ~500mA (within USB spec)
- Servos and stepper should not exceed 5V USB current limits simultaneously

---

## Diagram

> 📷 Wiring diagram image coming soon — see assembly photos in `docs/images/`
