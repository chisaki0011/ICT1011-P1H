# MSP430 TinyCircuit RC Project

## Project purpose

As people grow older, they may experience changes in cognitive abilities and
physical mobility, including reduced mobility in their hands and fingers.
Traditional exercises can also become repetitive. This project explores an
interactive racing-car game that lets an older adult control a car using real
hand movements rather than a conventional steering wheel or controller.

The activity is intended to encourage cognitive stimulation and hand mobility
through enjoyable gameplay. It gives users an opportunity to practise hand-eye
coordination, reaction time, concentration, and decision-making.

**This project is not intended to prevent, diagnose, or cure dementia or any
other medical condition.**

## How it works

The system has two TinyCircuits boards and an MSP430 LaunchPad:

1. **Wearable TinyCircuit (transmitter):** Worn on the hand or since a ring or on the wrist, it reads the X and
   Y axes of its BMA250 accelerometer. Hand tilts are sent over Bluetooth Low
   Energy (BLE) as text values such as `X12,Y-34\n`.
2. **Data-receiver TinyCircuit:** Scans for BLE devices and connects to the
   wearable. It parses the X and Y values and forwards them to the MSP430 over
   I²C. The receiver sends a six-byte packet: signed 16-bit X, Y, and Z values,
   each little-endian. Since Z is not sent by the wearable, the receiver sends
   Z as zero.
3. **MSP430 LaunchPad:** Acts on the received accelerometer values. Its firmware
   maps tilt to motor and steering-servo PWM signals, so the car responds to
   the user's hand movement. If fresh input stops arriving, a failsafe stops
   the drive and centers the steering.

```text
Hand movement
    ↓
Wearable TinyCircuit + BMA250 accelerometer
    ── X,Y text over BLE ──>
Data-receiver TinyCircuit
    ── X,Y (6-byte I²C packet) ──>
MSP430 LaunchPad
    ── PWM ──>
Drive motors and steering servo
    ↓
Racing car responds to the user's hand
```

The MSP430 firmware in `Motors.c` and `TwoWheels.c` contains the motor and
steering control. The files are firmware variants; check their axis mapping and
control behavior against the hardware setup before programming the LaunchPad.

## Repository contents

- `Emitter/Emitter.ino` — wearable TinyCircuit accelerometer sampling, display,
  and BLE data transmission.
- `Emitter/UART.ino` — BLE UART-style service and transmitter setup.
- `Receiver/Receiver.ino` — BLE scanning/connection, sample parsing, display,
  and I²C forwarding to the MSP430.
- `Motors/Motors.c` — MSP430 I²C receiver and PWM motor/steering control.
- `Motors/TwoWheels.c` — alternate MSP430 motor and steering control variant.

## Receiver controls

While the receiver is scanning, its display lists nearby devices. Use the
left-side buttons to move through the list and the upper-right button to
connect to a connectable device. The lower-right button restarts scanning.
Once connected, move the wearable hand to send a sample and control the car.
