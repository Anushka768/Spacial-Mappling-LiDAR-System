# Automated LiDAR Spatial Mapping System

**A mobile LiDAR spatial mapping system that uses a Time-of-Flight sensor mounted on a stepper motor to capture 360° distance scans and reconstruct indoor spaces in MATLAB.**

Developed by **Anushka Chauhan** for **COMPENG 2DX3 — Microprocessor Systems** at **McMaster University**, Winter 2026.

## Overview

This project uses a **VL53L1X Time-of-Flight sensor mounted on a 28BYJ-48 stepper motor**, an MSP432E401Y microcontroller, and MATLAB to map indoor environments. The stepper motor rotates the sensor through 360° to capture a vertical cross-section of the surrounding space. Combining scans collected at regular intervals along a straight path produces a 3D representation of the environment.

To automate the scanning process, I added a **keypad, DC motor, and wheels**. The user enters the desired number of scans, and the system captures those scans while moving forward in a straight line with consistent spacing between scan positions.
<img src="https://github.com/user-attachments/assets/d81035be-eb4b-4d0a-9176-26819b8d77ac"  alt="IMG_4635" width="400">
<img src="https://github.com/user-attachments/assets/8046ec6a-062c-452f-9d57-6858cda698c0"  alt="IMG_4636" width="400">




## Key Features

- **Automated multi-scan operation:** Enter the desired scan count using the keypad.
- **Motorized linear movement:** A DC motor and wheels move the platform between scan positions at consistent intervals.
- **360° distance scanning:** A stepper-mounted VL53L1X sensor captures 32 measurements per rotation, spaced 11.25° apart.
- **Automatic unwind:** The stepper motor returns to its starting orientation after each scan to prevent cable tangling.
- **Embedded data collection:** The microcontroller acquires distance readings over I2C and stores scan data onboard.
- **Serial data transfer:** Measurements are sent to the host computer over UART at 115200 baud.
- **3D visualization:** MATLAB converts distance measurements into Cartesian coordinates and displays an interactive reconstruction.
- **Status feedback:** Onboard LEDs indicate initialization, measurement capture, rotation, and data transmission.

## Demo & Results

To view the video demo: https://github.com/Anushka768/Spacial-Mappling-LiDAR-System/blob/main/LiDAR_Spatial_Mapping_Demo_Captioned.mp4 

<img src="https://github.com/user-attachments/assets/d2aeab3b-c69e-4eb8-92ba-ae2eb863e95f" alt="IMG_4572 2" width="500">
<img src="https://github.com/user-attachments/assets/edce8039-12ab-4350-a5ba-b20ee4ec0933" alt="IMG_4587"  width="500"  >


<img width="500"  src="https://github.com/user-attachments/assets/50f9db1e-cbf4-475d-9ff1-84037f15c3b5" >




## How It Works

1. **Set the scan count.** The user enters the number of scans using the keypad.
2. **Capture a spatial slice.** The stepper motor rotates the ToF sensor through 360°, pausing at each measurement position to collect a distance reading.
3. **Return to the starting orientation.** The stepper motor unwinds after the scan.
4. **Move to the next position.** The DC motor drives the wheeled platform forward by a consistent interval.
5. **Repeat.** The system continues until the requested number of scans has been collected.
6. **Reconstruct the environment.** Stored data is transferred to MATLAB, where the scans are processed and combined into a 3D model.

```text
Keypad → Requested scan count
                  ↓
          MSP432E401Y controller
            ├── Stepper motor + ULN2003 → Sensor rotation
            ├── VL53L1X over I2C        → Distance readings
            └── DC motor + wheels      → Linear movement
                  ↓
          Stored scan measurements
                  ↓ UART
           MATLAB → 3D reconstruction
```

## Hardware & Software

| Component | Role |
| --- | --- |
| MSP432E401Y | Coordinates sensing, motor control, data storage, and communication |
| VL53L1X Time-of-Flight sensor | Measures distance to surrounding surfaces |
| 28BYJ-48 stepper motor | Rotates the sensor through the scan plane |
| ULN2003 driver | Drives the stepper motor |
| DC motor and wheels | Move the platform along the scan path |
| Keypad | Accepts the requested number of scans |
| Onboard buttons and LEDs | Provide control and status feedback |
| Embedded C / Keil µVision | Microcontroller firmware development |
| MATLAB | Serial reception, data processing, and 3D plotting |

### Scanner Configuration

| Parameter | Configuration |
| --- | --- |
| Microcontroller clock | 22 MHz |
| Sensor communication | I2C at 100 kHz |
| Host communication | UART at 115200 baud |
| Measurements per scan | 32 |
| Angular interval | 11.25° |
| Scan coverage | 360° in the vertical YZ plane |
| Sensor timing budget | 50 ms |
| Logic / stepper supply | 3.3 V / 5 V |

## 3D Reconstruction

Each reading contains a distance and an associated angular position. The scan position supplies the third dimension:

```text
θ = k × (2π / 32), where k = 0, 1, ..., 31
x = n × Δx
y = d × cos(θ)
z = d × sin(θ)
```

Here, `d` is the measured distance, `n` is the zero-based scan index, and `Δx` is the spacing between scan positions. The X-axis follows the platform's travel direction, while the YZ-plane represents each vertical scan. The displacement used for reconstruction must match the physical spacing between scans.

The MATLAB processing workflow receives and parses serial measurements, filters out-of-range readings and abrupt spikes using neighboring samples, and plots the reconstructed points and connected scan outlines with `plot3()`. The resulting view can be rotated to inspect the geometry from different angles.

## Running the System

1. Assemble and power the scanner and wheeled platform, keeping the sensor's rotating field of view clear.
2. Connect the MSP432E401Y to the host computer and load the firmware using Keil µVision.
3. Position the platform at the start of a straight scan path.
4. Enter the desired scan count using the keypad and start the automated sequence.
5. Allow the system to complete the scans and movement between positions.
6. Open the MATLAB receiver script and select the correct serial port at **115200 baud**.
7. When MATLAB prompts for data, press **PJ1** to transmit the stored measurements and generate the 3D visualization.

## Design Considerations

- **Spatial resolution:** With 32 samples per revolution, small features between angular samples may be missed.
- **Movement consistency:** Straight travel and repeatable scan spacing affect how accurately the slices align in the final model.
- **Mechanical timing:** Stepper rotation and unwinding contribute significantly to the time needed for each scan cycle.
- **Sensor visibility:** The platform, wiring, and mounting structure must remain clear of the sensor's view.
- **Data filtering:** Replacing noisy readings can improve continuity but may also smooth real edges in the environment.

## Project Files

The project report, embedded C firmware, and MATLAB visualization script will be added to this repository.

## Author

**Anushka Chauhan**  
COMPENG 2DX3 — Microprocessor Systems  
McMaster University · Winter 2026
