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

# Alpha 2130 — 2026 Robot Code

This is the code for Alpha 2130's Robot File Explorer

Built with **WPILib** command-based Java, **YAGSL** swerve drive, **PathPlanner** autos, and **PhotonVision** for AprilTag targeting.

## Subsystems

- **Swerve Drive** — Swerve drivetrain
- **Launcher** — Dual Stationary Shooters
- **Intake** — Intake roller with dump mode
- **Intake Pivot** — Pivot arm with position control
- **Agitator** — Feeds balls into the launcher
- **Target Lock** — PhotonVision AprilTag alignment

## Controls (Xbox Controller) (Port 0)

| Input | Action |
|---|---|
| Left stick | Move In Cardinal Directions |
| Right stick X | Rotate |
| A | Move Intake Down From Up Postion |
| B | Intake |
| X | Main Shoot Button (Toggles Launcher-Agitator-Intake) |
| Y | AprilTag target lock |
| D-pad Up | Agitator toggle |
| D-pad Down | Launcher max speed |
| D-pad Left / Right | Pivot manual (Feather It Up And Down While Shooting) |
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
