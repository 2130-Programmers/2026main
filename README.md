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

Robot code for **Team 2130** (FIRST Robotics Competition 2026).

Built with **WPILib** command-based Java, **YAGSL** swerve drive, **PathPlanner** autos, and **PhotonVision** for AprilTag targeting.

## Subsystems

- **Swerve Drive** — YAGSL swerve drivetrain
- **Launcher** — Dual-motor ball shooter
- **Intake** — Intake roller with dump mode
- **Intake Pivot** — Pivot arm with position control
- **Agitator** — Feeds balls into the launcher
- **Target Lock** — PhotonVision AprilTag alignment

## Controls (Xbox Controller)

| Input | Action |
|---|---|
| Left stick | Drive (translate) |
| Right stick X | Rotate |
| A | Pivot toggle (94.5°) |
| B | Intake + pivot hold-down |
| X | Launcher, agitator, and intake |
| Y | AprilTag target lock |
| D-pad Up | Agitator toggle |
| D-pad Down | Launcher max speed |
| D-pad Left / Right (held) | Pivot manual |
| Left bumper | Intake dump |
| Right bumper | Zero gyro |

## Build & Deploy

Requires [WPILib 2026](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html).

```bash
./gradlew build    # compile
./gradlew deploy   # deploy to RoboRIO
```

Select autonomous routines from the **Auto Selector** on SmartDashboard.

## CAD

[SolidWorks assembly on Google Drive](https://drive.google.com/file/d/16M2vMvo7n1WblnARNBPhV3Jx4Lu4pAZf/view?usp=sharing)

## License

Code by **Team 2130**. WPILib is licensed under the [WPILib BSD License](WPILib-License.md).
