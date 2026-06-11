# Planetary-DrillRover-Teleop

An open-source, IPEx-inspired planetary rover analogue for sand-field mobility, drill-assisted soil interaction, and multimodal teleoperation experiments.

This repository contains the mechanical design files, embedded firmware, Raspberry Pi software, data logging tools, and supporting documentation for a compact four-wheel skid-steer rover prototype. The platform is designed as a reproducible terrestrial analogue for space robotics education, prototyping, field testing, and human-in-the-loop control research.

> This project is a terrestrial research prototype inspired by publicly available IPEx/RASSOR-style excavation rover concepts. It is not a flight-ready spacecraft and is not an official NASA project.

---

## 1. Project Overview

The rover combines:

- Four-wheel skid-steer mobility
- Large-diameter modular wheels with replaceable tread segments
- Drill-assisted soil loosening
- Rotating collection-disc mechanism
- Arm-assisted recovery and pit-escape capability
- STM32F407IGT6-based real-time embedded control
- Raspberry Pi 5 high-level computing layer
- Gesture-based teleoperation using an Ethernet camera
- Conventional radio control through FS-i6X / iA6B SBUS receiver
- Optional Bluetooth smart-terminal command interface
- Serial telemetry logging for field demonstrations

The prototype is intended for laboratory and sand-field experiments rather than vacuum, reduced-gravity, or radiation environments.

---

## 2. Repository Structure

```text
Planetary-DrillRover-Teleop/
├── firmware/
│   └── stm32f407igt6_rover_controller/
│       ├── Drivers/
│       ├── Inc/
│       ├── Src/
│       ├── MDK-ARM/
│       ├── application/
│       ├── bsp/
│       ├── boards/
│       ├── can.ioc
│       └── keilkill.bat
│
├── mechanical/
│   └── solidworks_source/
│       └── SolidWorks source files for the rover mechanical design
│
├── software/
│   └── raspberry_pi/
│       ├── bluetooth_bridge/
│       ├── data_logger/
│       └── gesture_control/
│
└── README.md
````

Planned or recommended folders for future release:

```text
electronics/
├── wiring_diagrams/
└── control_architecture/

data/
├── example_logs/
└── processed_plots/

docs/
├── assembly_guide/
├── operation_guide/
└── troubleshooting/
```

---

## 3. Hardware Architecture

### High-Level Computer

- Raspberry Pi 5
- Basler gesture-recognition camera connected through Ethernet
- USB-to-TTL serial interface for communication with the STM32 controller
- Optional Bluetooth smart-terminal bridge

### Embedded Controller

- STM32F407IGT6 embedded controller
- Real-time chassis control
- SBUS receiver decoding
- CAN actuator communication
- UART command reception and telemetry output

### Communication Interfaces

| Interface  | Function                                          |
| ---------- | ------------------------------------------------- |
| UART6      | Raspberry Pi command input to STM32               |
| UART1      | STM32 telemetry / feedback output to Raspberry Pi |
| UART3      | SBUS input from iA6B receiver                     |
| CAN1       | DJI M3508 chassis motor network                   |
| CAN2       | Soil-working actuator network                     |
| Ethernet   | Gesture camera to Raspberry Pi                    |
| USB serial | Optional Bluetooth terminal bridge                |

### Actuator Network

| Actuator      | Interface | Function                                          |
| ------------- | --------- | ------------------------------------------------- |
| 4 × DJI M3508 | CAN1      | Four-wheel skid-steer chassis drive               |
| DM4340 ID 1   | CAN2      | Left collection-disc drive                        |
| DM4340 ID 2   | CAN2      | Right collection-disc drive                       |
| DM4340 ID 3   | CAN2      | Collection-module arm actuator                    |
| ZDT57         | CAN2      | Lead-screw drive for vertical drill-module motion |
| ZDT42         | CAN2      | Drill rotation actuator                           |

### Power System

- Main power bus: 24 V
- Raspberry Pi power: 24 V to 5 V buck converter
- Motors and actuator drivers are powered from the 24 V system according to their voltage requirements

---

## 4. Firmware

The STM32 firmware is located in:

```text
firmware/stm32f407igt6_rover_controller/
```

Main functions include:

- Four-wheel chassis speed control
- CAN1 communication with DJI M3508 motors
- CAN2 communication with DM4340 and ZDT closed-loop stepper actuators
- FS-i6X / iA6B SBUS receiver decoding
- Raspberry Pi serial command parsing
- Command arbitration between radio control and vision-based control
- Telemetry output for experiment logging

The project is currently configured for Keil MDK-ARM and STM32CubeMX.

---

## 5. Raspberry Pi Software

The Raspberry Pi software is located in:

```text
software/raspberry_pi/
```

### `gesture_control/`

Gesture-recognition program for converting camera-based hand gestures into high-level rover commands.

### `bluetooth_bridge/`

Optional Bluetooth-to-serial bridge for smart-terminal command input.

### `data_logger/`

Python-based serial data logger for recording STM32 telemetry during field demonstrations.

Typical logged data include:

- Wheel target speed
- Wheel measured speed
- Arm feedback
- Drill command state
- Collection-disc command state
- Control source
- Remote-control channel values

---

## 6. Mechanical Design

The mechanical source files are located in:

```text
mechanical/solidworks_source/
```

This folder contains the original SolidWorks source files, including part files and assembly files.

Important file types:

| File type        | Meaning                     |
| ---------------- | --------------------------- |
| `.SLDPRT`        | SolidWorks part file        |
| `.SLDASM`        | SolidWorks assembly file    |
| `.SLDDRW`        | SolidWorks drawing file     |
| `.STEP` / `.STP` | General CAD exchange format |
| `.STL`           | Mesh format for 3D printing |

For best compatibility, future releases should include exported STEP files for the main assembly and key modules, as well as STL files for printable parts.

---

## 7. Demonstration Scenarios

The rover is evaluated in three representative sand-field scenarios:

1. **Flat sand demonstration**
   Basic mobility, drill-assisted soil loosening, and collection mechanism operation.

2. **Slope traversal demonstration**
   Slope mobility with arm posture and collection-disc assistance.

3. **Pit escape demonstration**
   Recovery behavior using short bursts of mobility, drill operation, arm deployment, and collection-disc actuation.

Processed plots and example logs will be placed in:

```text
data/example_logs/
data/processed_plots/
```

---

## 8. Getting Started

### Clone the repository

```bash
git clone https://github.com/ZhenzheXu/Planetary-DrillRover-Teleop.git
cd Planetary-DrillRover-Teleop
```

### Open STM32 firmware

Open the Keil project inside:

```text
firmware/stm32f407igt6_rover_controller/MDK-ARM/
```

or open the CubeMX configuration file:

```text
firmware/stm32f407igt6_rover_controller/can.ioc
```

### Run Raspberry Pi scripts

Raspberry Pi scripts are organized under:

```text
software/raspberry_pi/
```

Detailed setup instructions will be added in the operation guide.

---

## 9. Status

This repository is under active development.

Current release status:

- [x] STM32F407IGT6 firmware structure added
- [x] Raspberry Pi software folders created
- [x] SolidWorks mechanical source folder created
- [ ] STEP release files to be added
- [ ] STL printable parts to be added
- [ ] Wiring diagrams to be added
- [ ] Example field-test logs to be added
- [ ] Processed plots to be added
- [ ] Assembly and operation guides to be added

---

## 10. Citation

If you use this project in academic work, please cite the corresponding paper or this repository.

```bibtex
@misc{xu_planetary_drill_rover_2026,
  author       = {Xu, Zhenzhe},
  title        = {Planetary-DrillRover-Teleop: Open-source planetary drill rover teleoperation platform},
  year         = {2026},
  howpublished = {GitHub repository},
  url          = {https://github.com/ZhenzheXu/Planetary-DrillRover-Teleop}
}
```

---

## 11. License

A license file will be added before the public release.

Recommended structure:

- Firmware and software: MIT License or Apache-2.0
- Mechanical design files: CERN Open Hardware Licence or Creative Commons Attribution license

---

## 12. Safety Warning

This project is a research prototype intended for education, experimentation, and prototyping. It must be operated only by trained personnel in a controlled environment. Users are fully responsible for safe assembly, wiring, battery handling, and field operation. Do not operate the rover near people, unstable terrain, or hazardous conditions. The prototype is not intended for spaceflight, human-rated operation, or use in hazardous environments.
