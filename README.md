# QDD5010 Actuator

A quasi-direct-drive robot joint built from an MKS 5010 360 KV gimbal motor and a 3D-printed 6:1 planetary gearbox, run with SimpleFOC on an Arduino Mega.

## Hardware

- Arduino Mega 2560 with a SimpleFOC Shield v2.0.4
- MKS 5010 360 KV motor (7 pole pairs) and an MKS AS5600 magnetic encoder over I2C (Mega pins 20/21)
- 12 V supply
- Printed planetary gearbox, version 2: sun 12 teeth, ring 60 teeth, three planets, carrier output, 6:1

## Repository

| Path | What it is |
| --- | --- |
| `Code/QDD5010/QDD5010.ino` | Firmware: FOC current control with feedforward, angle / speed / torque modes, serial commands and telemetry |
| `Code/WebApp/` | Browser console: commands, live charts, a 3D view, contact detection and torque calibration |
| `CAD/V2/` | FreeCAD model of the V2 gearbox and calibration arm, with 3MF files ready to print |

## Firmware

Open `Code/QDD5010/QDD5010.ino` in the Arduino IDE (needs the Simple FOC library), build for the Mega and upload. The serial port runs at 115200 baud. Telemetry is off at boot; send `M` to start or stop it (20 Hz).

| Command | Does |
| --- | --- |
| `B<deg>` | Output angle |
| `A<deg>` | Motor angle |
| `S<rpm>` | Output speed |
| `R<rpm>` | Motor speed |
| `T<amps>` | Torque, as q-axis current |
| `Z` | Zero the output angle here |
| `E0` / `E1` | Disable / enable the motor |
| `C<amps>` | Current limit |
| `QV<volts>` | Voltage limit |
| `G`, `H` | Status and help |
| `M` | Telemetry on / off |

## Web console

Open `Code/WebApp/index.html` in Chrome or Edge (it uses Web Serial), click **Connect** and pick the Mega's port. If the browser blocks Web Serial on a local file, serve the folder instead, for example `python -m http.server 8765` in `Code/WebApp`, and open http://localhost:8765 (`start.bat` does this on Windows). **Simulator** runs the console without hardware.

Contact detection compares the measured current with what free motion needs (a learned friction map per 10° of output plus an acceleration term). Run **Learn free motion** once, with the arm free to swing a full turn, before relying on it.

## Printing

The 3MF files in `CAD/V2/` are the printed parts: base, ring, sun, three planets, carrier, output flange, bearing retainer, rotor adapter and the 100 mm calibration arm (bolts to the flange with 4 × M3 × 10).
