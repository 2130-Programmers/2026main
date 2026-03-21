package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.AgitatorSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.TargetLockSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

    // ─── Subsystems ───────────────────────────────────────────────────────────
    public final AgitatorSubsystem      agitator   = new AgitatorSubsystem();
    public final SwerveSubsystem        swerve     = new SwerveSubsystem();
    public final LauncherSubsystem      launcher   = new LauncherSubsystem();
    public final IntakeSubsystem        intake     = new IntakeSubsystem();
    public final IntakePivotSubsystem   pivot      = new IntakePivotSubsystem(95.0); // default 95°
    public final TargetLockSubsystem    targetLock = new TargetLockSubsystem(new PhotonCamera("Arducam_OV9281_USB_Camera"));

    // ─── Controller ───────────────────────────────────────────────────────────
    private final CommandXboxController m_driverController =
            new CommandXboxController(OperatorConstants.kDriverControllerPort);

    // ─── Dashboard ────────────────────────────────────────────────────────────
    private final SendableChooser<Command> autoChooser;
    private final SendableChooser<Boolean> drivingModeChooser = new SendableChooser<>();

    // Field2d widget — plots robot pose on a field diagram in SmartDashboard.
    // Add a "Field" widget in SmartDashboard to see the robot move in real time.
    private final Field2d field = new Field2d();

    // ─────────────────────────────────────────────────────────────────────────

    public RobotContainer() {

        // ── PathPlanner named commands ────────────────────────────────────────
        NamedCommands.registerCommand("PivotToggle",      new InstantCommand(pivot::toggle, pivot));
        NamedCommands.registerCommand("IntakeToggle",     new InstantCommand(intake::toggle, intake));
        NamedCommands.registerCommand("LauncherToggle",   new InstantCommand(launcher::toggle, launcher));
        NamedCommands.registerCommand("TargetLockToggle", new InstantCommand(targetLock::toggleLock));
        NamedCommands.registerCommand("AgitatorToggle",   new InstantCommand(agitator::toggle, agitator));
        NamedCommands.registerCommand("SetTargetDegrees",  new InstantCommand(() -> { pivot.setTargetDegrees(60); pivot.toggle(); }, pivot));
        NamedCommands.registerCommand("PivotAgitate",
            new RunCommand(pivot::startAgitate, pivot)
                .withTimeout(5.0)
                .finallyDo((interrupted) -> pivot.stopAgitate())
        );

        launcher.setTargetLock(targetLock);

        configureBindings();

        // ── Default drive command ─────────────────────────────────────────────
        swerve.setDefaultCommand(
            new RunCommand(() -> {
                double xSpeed = -m_driverController.getLeftY();
                double ySpeed = -m_driverController.getLeftX();
                double rot    = -m_driverController.getRightX();

                // Deadbands
                xSpeed = Math.abs(xSpeed) < 0.12 ? 0 : xSpeed;
                ySpeed = Math.abs(ySpeed) < 0.12 ? 0 : ySpeed;
                rot    = Math.abs(rot)    < 0.15 ? 0 : rot;

                // Scale translation
                xSpeed *= 4.0;
                ySpeed *= 4.0;

                // Target lock overrides rotation
                if (targetLock.isLocked()) {
                    rot = targetLock.getRotationSpeed();
                } else {
                    rot *= Math.PI * 1.3;
                }

                swerve.drive(xSpeed, ySpeed, rot, drivingModeChooser.getSelected());
            }, swerve)
        );

        // ── SmartDashboard ────────────────────────────────────────────────────
        drivingModeChooser.setDefaultOption("Robot Oriented", false);
        drivingModeChooser.addOption("Field Oriented", true);
        SmartDashboard.putData("Driving Mode",  drivingModeChooser);

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Selector", autoChooser);

        SmartDashboard.putData("Reset Gyro",    new InstantCommand(swerve::zeroGyro));

        // Register the Field2d widget — open "Field" in SmartDashboard to view it
        SmartDashboard.putData("Field", field);
    }

    // ─── Period init (called from Robot.java) ─────────────────────────────────

    /**
     * Called by Robot.java teleopInit().
     * Resets the vision initial pose flag and zeroes the gyro for the correct
     * alliance so field-oriented driving works from either side of the field.
     */
    public void teleopInit() {
      
    }

    /**
     * Called by Robot.java autonomousInit().
     * Resets the vision initial pose flag — PathPlanner will hard-set the pose
     * from the path's starting position on the first loop cycle.
     */
    public void autonomousInit() {
       
    }

    // ─── Periodic update (called from Robot.java robotPeriodic()) ────────────

    /**
     * Updates the Field2d widget with the latest robot pose.
     * Call this from Robot.java robotPeriodic() so it updates every loop cycle.
     */
    public void updateField() {
       
    }

    // ─── Bindings ─────────────────────────────────────────────────────────────

    private void configureBindings() {

        // A button: pivot to 94.5° and toggle
        m_driverController.a().onTrue(
            new InstantCommand(() -> {
                pivot.setTargetDegrees(94.5);
                pivot.toggle();
            }, pivot)
        );

         // D-pad UP held: pivot manual forward
         m_driverController.povUp()
            .onTrue(new InstantCommand(agitator::toggle, agitator));

        // D-pad right held: pivot manual forward
        m_driverController.povRight()
            .whileTrue(new RunCommand(pivot::manualForward, pivot))
            .onFalse(new InstantCommand(pivot::manualStop, pivot));

        // D-pad left held: pivot manual reverse
        m_driverController.povLeft()
            .whileTrue(new RunCommand(pivot::manualReverse, pivot))
            .onFalse(new InstantCommand(pivot::manualStop, pivot));

        // B button: intake toggle
        m_driverController.b().onTrue(new InstantCommand(intake::toggle, intake));

        // X button: launcher toggle
        m_driverController.x()
        .onTrue(new InstantCommand(launcher::toggle, launcher))
        .onTrue(new InstantCommand(agitator::toggle, agitator))
        .onTrue(new InstantCommand(intake::toggle, intake))
        ;

        // Y button: target lock toggle
        m_driverController.y().onTrue(new InstantCommand(targetLock::toggleLock));
    }

    // ─── Getters ──────────────────────────────────────────────────────────────

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public SwerveSubsystem getSwerve() {
        return swerve;
    }
}