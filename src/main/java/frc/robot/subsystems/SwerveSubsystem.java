package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.SwerveModule;

import java.io.File;
import java.io.IOException;

public class SwerveSubsystem extends SubsystemBase {

    private final SwerveDrive swerveDrive;

    public SwerveSubsystem() {
        SwerveDrive tempDrive;

        try {
            // Load JSON configs from deploy/swerve
            File configDir = new File(Filesystem.getDeployDirectory(), "swerve");
            tempDrive = new SwerveParser(configDir).createSwerveDrive(5.0); // max 5 m/s

            for (var module : tempDrive.getModules()) {
    System.out.println(module.configuration.name + " offset: " + module.getAbsolutePosition());
}
        } catch (IOException e) {
            e.printStackTrace();
            throw new RuntimeException("Failed to load swerve JSON configs!", e);
        }

        swerveDrive = tempDrive;

        // Put all drive motors in brake (better control)
        swerveDrive.setMotorIdleMode(true);
    }






    /**
     * Drive the robot (GYROLESS-SAFE VERSION).
     *
     * @param xSpeed forward/backward (m/s)
     * @param ySpeed left/right (m/s)
     * @param rot rotation (rad/s)
     * @param fieldOriented IGNORED when no gyro (we force robot-centric)
     */

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldOriented) {
        Translation2d translation = new Translation2d(xSpeed, ySpeed);

        // ===== FORCE ROBOT-ORIENTED (REQUIRED WITH NO GYRO) =====
        boolean useFieldOriented = false;

        swerveDrive.drive(
                translation,
                rot,
                useFieldOriented, // always robot-centric without a gyro
                true              // open-loop (matches your template)
        );
    }

@Override
public void periodic() {
    // Update odometry (existing)
    swerveDrive.updateOdometry();

    }
}

