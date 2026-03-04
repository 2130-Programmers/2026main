package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

    // Subsystems
    public final SwerveSubsystem swerve = new SwerveSubsystem();
    public final LauncherSubsystem launcher = new LauncherSubsystem();
    public final IntakeSubsystem intake = new IntakeSubsystem();
    public final IntakePivotSubsystem pivot = new IntakePivotSubsystem();
    // Controller
    private final CommandXboxController m_driverController =
            new CommandXboxController(OperatorConstants.kDriverControllerPort);

    public RobotContainer() {
        configureBindings();

        // Default drive command
        swerve.setDefaultCommand(
            new RunCommand(() -> {
                double xSpeed = -m_driverController.getLeftY();   // forward/back (inverted)
                double ySpeed = -m_driverController.getLeftX();   // strafe
                double rot = -m_driverController.getRightX();     // rotation (inverted)

                // Add deadbands so tiny joystick noise doesn't spin the robot
                xSpeed = Math.abs(xSpeed) < 0.12 ? 0 : xSpeed;
                ySpeed = Math.abs(ySpeed) < 0.12 ? 0 : ySpeed;
                rot    = Math.abs(rot)    < 0.15 ? 0 : rot;

                // Scale to reasonable speeds
                xSpeed *= 4.0;   // m/s
                ySpeed *= 4.0;
                rot    *= Math.PI * 0.6; // slow rotation a bit

                // IMPORTANT: true = ROBOT-ORIENTED (safe with no gyro)
                swerveDrive.drive(xSpeed, ySpeed, rot, true);
            }, swerve)
        );
    }

    private void configureBindings() {
        // A button: pivoty
         m_driverController.a().onTrue(new InstantCommand(pivot::toggle, pivot));

        // B button: Keep your existing code or remove if not needed
         m_driverController.b().onTrue(new InstantCommand(intake::toggle, intake));

        // X button: Toggle launcher on/off
        m_driverController.x().onTrue(new InstantCommand(launcher::toggle, launcher));
    }

    /**
     * Autonomous command placeholder
     */
   public Command getAutonomousCommand() {
    return swerve.getAutonomousCommand("New Auto"); // use `swerve`, not `drivebase`
}
    
    public SwerveSubsystem getSwerve() {
        return swerve;
    }
}