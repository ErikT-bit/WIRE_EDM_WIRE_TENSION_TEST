# Wire EDM Wire-Tension Calibration Bench

This repository contains the code and setup notes for a bench-top wire-tension calibration system used to tune an electric brake for a wire EDM wire-feed system.

The bench pulls a short closed loop of wire with a stepper-driven capstan while a second motor applies braking resistance through a moteus controller. A load-cell pulley measures force in the wire path. MATLAB coordinates the test, sends commands to the Arduino, launches the moteus logger, displays live data, and stores saved runs for later analysis.

This repository documents the **test and calibration bench**. It is separate from the later Raspberry Pi / LinuxCNC production controller.

---

## Hardware Used

The original bench used:

- Windows computer running MATLAB, Python, and the Arduino IDE
- Arduino Uno
- HX711 load-cell amplifier
- 5 kgf load cell
- incremental quadrature encoder
- DM556 stepper driver
- NEMA 23 stepper motor used as the wire-pull motor
- driver capstan pulleys with multiple diameters
- load-cell pulley
- moteus r4.11 motor controller
- motor connected to the moteus controller and used as the electric brake
- mjbots `mjcanfd-usb-1x` adapter for laptop-to-moteus communication
- suitable motor power supplies
- short closed wire loop
- mechanical frame and pulley mounts

Another build can use different motors, pulley sizes, encoder resolution, load-cell capacity, ports, and pin assignments. Review and update the setup-specific values before running the code.

---

## Main Workflow Files

```text
run_wire_tension_dashboard.m
damper_kd_csv_logger.py
wire_tension_arduino.ino
```

### `run_wire_tension_dashboard.m`

Runs the experiment from MATLAB. It:

- prompts the user for test settings
- calculates the required driver RPM from the selected linear wire speed and pulley diameter
- connects to the Arduino over a serial port
- sends a sequence of `RPM=` commands to ramp the pull motor from zero speed to the requested test speed
- launches `damper_kd_csv_logger.py`
- displays live plots
- stops the motor at the end of the run
- stores saved data for later analysis

### `damper_kd_csv_logger.py`

Runs from Python and communicates with the moteus controller. It:

- applies the selected damping value
- logs moteus telemetry during the test
- records data used to characterize braking behavior as wire speed changes

### `wire_tension_arduino.ino`

Runs on the Arduino Uno. It:

- reads the HX711 load-cell amplifier
- filters load-cell readings with an exponential moving average
- reads the encoder
- calculates measured RPM
- uses encoder feedback with a PI controller to correct the pull-motor speed
- controls the DM556 stepper driver
- receives serial commands from MATLAB
- returns CSV-style telemetry to MATLAB

The speed ramp is coordinated by MATLAB. MATLAB sends increasing `RPM=` commands during the ramp, and the Arduino uses encoder feedback to track each commanded speed.

---

## Load-Cell Calibration Files

```text
run_loadcell_calibration.m

HX711_LoadCell_Calabration_in_MATLAB/
└── HX711_LoadCell_Calabration_in_MATLAB.ino
```

These files are used to determine the HX711 scale factor before collecting test data.

The original calibration process used repeated checks near:

```text
2 kg
4 kg
```

This prioritized accuracy near the expected operating range while also checking the response at a higher load.

---

## Helpful Standalone Utilities

These utilities are included because they were useful for testing, validation, and troubleshooting:

```text
damper_kd_monitor.py
damper_kd_stream_file.py
plot_encoder_rpm_live.m
run_wire_tension_live.m
```

They are not required by the main MATLAB dashboard, but they are useful when validating individual parts of the system.

---

## Software Requirements

Install:

- MATLAB
- Python 3
- Arduino IDE
- Arduino `HX711` library
- Arduino `AccelStepper` library
- mjbots moteus Python tools and drivers for the CAN-FD adapter
- any Python packages imported by the moteus scripts

---

## Original Arduino Pin Assignments

These were the pin assignments used on the original bench:

| Function | Arduino Uno Pin |
|---|---:|
| Encoder A | D2 |
| Encoder B | D3 |
| HX711 DT | D5 |
| HX711 SCK | D6 |
| DM556 STEP | D8 |
| DM556 DIR | D9 |
| DM556 ENABLE | D10 |

These values are examples, not universal requirements. Change the pin definitions in `wire_tension_arduino.ino` if your wiring is different.

---

## Setup-Specific Values to Verify

Review these values before running the bench:

| Setting | Original Bench Example | What to Do on Another Build |
|---|---:|---|
| Arduino serial port | `COM16` during one test session | Use the port assigned by your computer |
| HX711 calibration factor | `364.0` in the final Arduino sketch | Recalibrate the installed load cell |
| Load-cell force divider | `2.0` | Verify from the actual pulley geometry |
| Encoder counts per revolution | `2000` | Match the installed encoder and decoding mode |
| DM556 microstep setting | `16` microsteps | Match the DM556 DIP-switch configuration |
| Driver pulley diameter | Entered for each run | Enter the pulley actually installed |
| Moteus ID | `1` | Change if the controller uses another ID |
| Python path | Computer-dependent | Update the MATLAB launcher if needed |
| Output directory | Computer-dependent | Update the MATLAB save path if needed |

Do not copy another computer's COM port or file paths without checking them.

---

## Mechanical Measurement Concept

The wire travels over a pulley connected to the load cell.

The original workflow used a default force divider of:

```text
2.0
```

The estimated wire tension was calculated as:

```text
estimated wire tension = measured load-cell force / divider
```

The correct divider depends on the actual wire path and pulley geometry. Verify it experimentally for a different mechanical arrangement.

---

## Arduino Telemetry

The final Arduino sketch returns CSV-style telemetry in this order:

```text
time_ms,state_code,record_ready,target_rpm_active,target_rpm_final,meas_rpm,cmd_rpm,load_g_raw,load_g_filt,tension_g,tension_N,enc_count
```

Keep the Arduino and MATLAB column definitions synchronized if you modify this format.

---

## Workflow

### 1. Assemble the wire loop

Route a short closed loop of wire through:

- the driver capstan
- the moteus brake capstan
- the load-cell pulley

Confirm that the wire tracks correctly and that the load-cell pulley moves freely.

### 2. Wire the Arduino-side hardware

Connect:

- load cell to HX711
- HX711 to Arduino
- encoder outputs to Arduino
- Arduino STEP, DIR, and ENABLE outputs to the DM556
- DM556 to the stepper motor
- Arduino to the computer by USB

### 3. Connect the moteus brake

Connect:

- brake motor to the moteus controller
- moteus controller to its motor power supply
- moteus controller to the computer through the supported CAN-FD adapter

### 4. Update setup-specific values

Before uploading or running the code:

- verify Arduino pin assignments
- enter the correct encoder counts per revolution
- match the DM556 microstep setting
- calibrate the load cell
- confirm the Arduino serial port
- confirm the moteus ID
- update computer-specific paths
- enter the installed pulley diameter

### 5. Calibrate the load cell

Upload:

```text
HX711_LoadCell_Calabration_in_MATLAB.ino
```

Use known reference weights with:

```matlab
run_loadcell_calibration
```

to determine the appropriate HX711 scale factor.

### 6. Upload the final test Arduino sketch

Upload:

```text
wire_tension_arduino.ino
```

to the Arduino Uno.

### 7. Run the MATLAB dashboard

In MATLAB, change into the folder containing the scripts and run:

```matlab
run_wire_tension_dashboard
```

Follow the prompts to enter the test settings.

---

## Suggested Repository Layout

```text
Wire-EDM-Tension-Calibration/
├── README.md
└── code/
    └── Kd calibration Velocity vs Tension test code/
        ├── run_wire_tension_dashboard.m
        ├── run_loadcell_calibration.m
        ├── run_wire_tension_live.m
        ├── plot_encoder_rpm_live.m
        ├── damper_kd_csv_logger.py
        ├── damper_kd_monitor.py
        ├── damper_kd_stream_file.py
        ├── wire_tension_arduino.ino
        └── HX711_LoadCell_Calabration_in_MATLAB/
            └── HX711_LoadCell_Calabration_in_MATLAB.ino
```

---

## Notes for Reproducing the Bench

- Start with low motor speed and low braking resistance when validating a new build.
- Recalibrate the installed load cell.
- Verify all Arduino pins before wiring.
- Match the DM556 DIP-switch settings to the Arduino sketch.
- Verify encoder resolution and quadrature decoding mode.
- Confirm the wire-tension divider for the actual pulley geometry.
- Keep computer-specific ports and file paths out of shared assumptions.
