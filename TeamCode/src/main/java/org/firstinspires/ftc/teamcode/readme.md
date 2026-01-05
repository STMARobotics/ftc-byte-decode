# FTC Byte Decode - Team Code

This document provides an overview of the codebase structure for the FTC Byte Decode robot.

## Project Structure

```
teamcode/
├── opmodes/           # TeleOp and Autonomous operation modes
│   ├── teleop/        # Driver-controlled operation modes
│   └── auto/          # Autonomous operation modes
├── subsystems/        # Robot subsystem classes (drivetrain, arm, intake, etc.)
├── commands/          # Command-based architecture commands
├── math/              # Math related to interpolation
├── pedropathing/      # Code related to the pedropathing library
└── constants/         # Robot constants and configuration values
```

## Key Components

### OpModes
- **TeleOp**: Driver-controlled modes for competition and testing
- **Autonomous**: Pre-programmed routines for the autonomous period

### Subsystems
Individual hardware components abstracted into manageable classes:
- Drivetrain (mecanum/tank drive)
- Turret system
- Shooting system
- Intake system
- Sensors

### Utilities
- PID controllers
- Coordinate systems
- Logging utilities

## Getting Started

1. Clone the repository
2. Open in Android Studio
3. Connect to the Robot Controller
4. Deploy and run

## Naming Conventions

- OpModes: `[Type]_[Description]` (e.g., `TeleOp_Main`, `Auto_RedLeft`)
- Subsystems: `[Component]Subsystem` (e.g., `DrivetrainSubsystem`)
- Constants: `UPPER_SNAKE_CASE`
