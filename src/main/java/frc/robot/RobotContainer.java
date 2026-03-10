package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.TargetLockSubsystem;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

    // Subsystems
    public final SwerveSubsystem swerve = new SwerveSubsystem();
    public final LauncherSubsystem launcher = new LauncherSubsystem();
    public final IntakeSubsystem intake = new IntakeSubsystem();
    public final IntakePivotSubsystem pivot = new IntakePivotSubsystem();
    public final TargetLockSubsystem targetLock = new TargetLockSubsystem(new PhotonCamera("Arducam_OV9281_USB_Camera"));

    // Controller
    private final CommandXboxController m_driverController =
            new CommandXboxController(OperatorConstants.kDriverControllerPort);

    public RobotContainer() {
        // Wire targetLock into launcher so periodic() always gets fresh distance
        launcher.setTargetLock(targetLock);

        configureBindings();

        // Default drive command
        swerve.setDefaultCommand(
            new RunCommand(() -> {
                double xSpeed = -m_driverController.getLeftY();
                double ySpeed = -m_driverController.getLeftX();
                double rot    = -m_driverController.getRightX();

                // Deadbands
                xSpeed = Math.abs(xSpeed) < 0.12 ? 0 : xSpeed;
                ySpeed = Math.abs(ySpeed) < 0.12 ? 0 : ySpeed;
                rot    = Math.abs(rot)    < 0.15 ? 0 : rot;

                // Scale to speeds
                xSpeed *= 4.0;
                ySpeed *= 4.0;

                // If target lock is active and a target is visible, use PID rotation
                // Otherwise use normal driver rotation
                if (targetLock.isLocked()) {
                    rot = targetLock.getRotationSpeed();
                } else {
                    rot *= Math.PI * 1.0;
                }

                swerve.drive(xSpeed, ySpeed, rot, true);
            }, swerve)
        );
    }

    private void configureBindings() {
        // A button: pivot toggle
        m_driverController.a().onTrue(new InstantCommand(pivot::toggle, pivot));

        // B button: intake toggle
        m_driverController.b().onTrue(new InstantCommand(intake::toggle, intake));

        // X button: launcher toggle
        m_driverController.x().onTrue(new InstantCommand(launcher::toggle, launcher));

        // Y button: toggle target lock on/off
        m_driverController.y().onTrue(new InstantCommand(targetLock::toggleLock));
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("New Auto");
    }

    public SwerveSubsystem getSwerve() {
        return swerve;
    }
}