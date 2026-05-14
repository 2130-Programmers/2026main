```
 .d8888b.   d888    .d8888b.   .d8888b. 
d88P  Y88b d8888   d88P  Y88b d88P  Y88b
       888   888        .d88P 888    888 
     .d88P   888       8888"  888    888 
 .od888P"    888        "Y8b. 888    888 
d88P"        888   888    888 888    888 
888"         888   Y88b  d88P Y88b  d88P 
888888888  8888888  "Y8888P"   "Y8888P"  

                TEAM ALPHA 2130
```

# Alpha 2130 — 2026 FRC Robot Code

> **Team 2130 | FIRST Robotics Competition 2026**  
> Command-based Java robot code for **Robo Reel**, built on WPILib + YAGSL swerve drive.

---

## Overview

This repository contains the full robot control code for Team 2130's 2026 competition robot. The codebase uses WPILib's command-based framework with YAGSL (Yet Another Generic Swerve Library) for drivetrain control, PathPlannerLib for autonomous path following, and PhotonVision for AprilTag-based vision targeting.

---

## CAD Files

Here is the google drive link to the SolidWorks Assembly Of File Explorer https://drive.google.com/file/d/16M2vMvo7n1WblnARNBPhV3Jx4Lu4pAZf/view?usp=sharing

## Subsystems

| Subsystem | Class | Description |
|---|---|---|
| **Swerve Drive** | `SwerveSubsystem` | YAGSL-powered swerve drivetrain with heading correction and cosine compensation |
| **Launcher** | `LauncherSubsystem` | Dual-motor ball launcher with distance-scaled speed and right-side trim |
| **Intake** | `IntakeSubsystem` | Single-motor intake roller with forward and dump (reverse) modes |
| **Intake Pivot** | `IntakePivotSubsystem` | Dual-motor pivot arm with PID position control, manual override, hold-down mode, and ball agitation |
| **Agitator** | `AgitatorSubsystem` | Dual-motor ball agitator to feed the launcher |
| **Target Lock** | `TargetLockSubsystem` | PhotonVision AprilTag tracker with PID rotation control and slew-rate limiting |

---

## Controls (Xbox Controller — Port 0)

| Input | Action |
|---|---|
| **Left Stick** | Translate (forward/strafe) |
| **Right Stick X** | Rotate |
| **A Button** | Pivot to 94.5° and toggle |
| **B Button** | Toggle intake + pivot hold-down mode |
| **X Button** | Toggle launcher, agitator, and intake simultaneously |
| **Y Button** | Toggle AprilTag target lock |
| **D-Pad Up** | Toggle agitator |
| **D-Pad Down** | Toggle launcher max speed mode |
| **D-Pad Left (held)** | Pivot manual reverse |
| **D-Pad Right (held)** | Pivot manual forward |
| **Left Bumper** | Intake dump (reverse) toggle |
| **Right Bumper** | Zero gyro |

**Deadbands:** Left stick < 12%, Right stick X < 15%  
**Drive speed cap:** 4.0 m/s translation, ~4.08 rad/s rotation (or PID-controlled when target locked)

---

## Autonomous

Autonomous routines are selected via SmartDashboard using PathPlannerLib's auto chooser.

### Available Autos

| Auto | Description |
|---|---|
| `CenterBack` | Center start, back up |
| `RightSide` | Right-side starting position |
| `RightTrench` | Right trench run |
| `Simple center` | Basic center routine |

### PathPlanner Named Commands

These named commands are registered and can be used in any PathPlanner auto:

- `PivotToggle` — Toggle pivot position
- `IntakeToggle` — Toggle intake + pivot hold-down
- `LauncherToggle` — Toggle launcher
- `ZeroGyro` — Reset gyro heading
- `TargetLockToggle` / `LauncherTarget` — Toggle AprilTag target lock
- `AgitatorToggle` — Toggle agitator
- `SetTargetDegrees` — Set pivot to 60° and toggle
- `PivotAgitate` — Run pivot agitation for 5 seconds

---

## Vision

The robot uses **PhotonVision** with an `Arducam_OV9281_USB_Camera` for AprilTag detection.

Target Lock (`TargetLockSubsystem`) tracks valid AprilTag IDs `[2, 8, 10, 18, 21, 26]` and uses a PID controller to align the robot rotationally. When locked, the rotation output overrides manual stick input in the drive command.

**PID tuning:** kP = 0.03, kI = 0.0, kD = 0.001 with a 1.5° tolerance and 3.0 rad/s max output.

---

## Vendor Dependencies

| Library | Version |
|---|---|
| WPILib / GradleRIO | 2026.2.1 |
| PathPlannerLib | 2026.1.2 |
| YAGSL | 2026.1.30 |
| REVLib (SparkMax) | latest |
| CTRE Phoenix 5 | 5.36.0 (replay) |
| CTRE Phoenix 6 | 26.1.1 (replay) |
| PhotonVision | latest |
| Redux Lib | 2026.1.1 |
| ThriftyLib | 2026 |
| Studica | latest |
| Playing With Fusion | 2026 |

---

## SmartDashboard

The following widgets are published to SmartDashboard:

- **Auto Selector** — Dropdown to choose autonomous routine
- **Driving Mode** — Toggle between Robot-Oriented and Field-Oriented drive
- **Field** — Live robot pose visualization (Field2d widget)
- **Intake** — Boolean indicator for intake running state
- **Reset Gyro** — Sendable button to zero the gyro

---

## Building & Deploying

### Prerequisites

- [WPILib 2026 installation](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html) (includes VS Code, Gradle, Java 17)
- Robot connected to the same network or USB tether

### Build

```bash
./gradlew build
```

### Deploy to RoboRIO

```bash
./gradlew deploy
```

### Simulate (Desktop)

```bash
./gradlew simulateJava
```

> **Note:** Desktop simulation support is currently disabled (`includeDesktopSupport = false` in `build.gradle`). Enable it before running simulation.

---

## Project Structure

```
src/main/java/frc/robot/
├── Main.java                        # Entry point
├── Robot.java                       # Timed robot lifecycle
├── RobotContainer.java              # Subsystem wiring + button bindings
├── Constants.java                   # Robot-wide constants
├── commands/
│   ├── Autos.java                   # Autonomous command factory
│   └── ExampleCommand.java
└── subsystems/
    ├── SwerveSubsystem.java         # YAGSL swerve drive
    ├── LauncherSubsystem.java       # Ball launcher
    ├── IntakeSubsystem.java         # Ball intake
    ├── IntakePivotSubsystem.java    # Intake pivot arm
    ├── AgitatorSubsystem.java       # Ball agitator
    └── TargetLockSubsystem.java     # PhotonVision target lock

src/main/deploy/
├── swerve/                          # YAGSL swerve config (modules, drive params)
└── pathplanner/                     # Auto paths and PathPlanner settings
```

---

## License

Robot code is developed by **Team 2130**. WPILib components are licensed under the [WPILib BSD License](WPILib-License.md).
