package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.io.File;
import java.io.IOException;

public class SwerveSubsystem extends SubsystemBase {

    private final SwerveDrive swerveDrive;
    public SwerveSubsystem() {
        try {
            File configDir = new File(Filesystem.getDeployDirectory(), "swerve");
            swerveDrive = new SwerveParser(configDir).createSwerveDrive(4.5);

            // ✅ Use gyro for rotation, wheel encoders for translation
            swerveDrive.setHeadingCorrection(true);  // Corrects heading drift using gyro
            swerveDrive.setCosineCompensator(true);   // Smoother wheel control

            for (var module : swerveDrive.getModules()) {
                System.out.println(module.configuration.name + " offset: " + module.getAbsolutePosition());
            }
        } catch (IOException e) {
            e.printStackTrace();
            throw new RuntimeException("Failed to load swerve JSON configs!", e);
        }
        setupPathPlanner();
    }
      /**
   * Setup AutoBuilder for PathPlanner.
   */
public void zeroGyro() {
    swerveDrive.zeroGyro();
}

public Rotation2d getHeading() {
    return swerveDrive.getYaw(); // Pure gyro reading
}

public void resetOdometry(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
}
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative)
{
    ChassisSpeeds speeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, swerveDrive.getYaw())
        : new ChassisSpeeds(xSpeed, ySpeed, rot);

    swerveDrive.setChassisSpeeds(speeds);
}
public void setupPathPlanner()
{
    
    System.out.println(">>> setupPathPlanner START");
    RobotConfig config;
    try
    {
        config = RobotConfig.fromGUISettings();
        System.out.println(">>> RobotConfig loaded OK");
    } catch (Exception e)
    {
        System.out.println(">>> RobotConfig FAILED");
        e.printStackTrace();
        return;
    }

    try
    {
        
        System.out.println(">>> Calling AutoBuilder.configure");
        AutoBuilder.configure(
            swerveDrive::getPose,
            swerveDrive::resetOdometry,
            swerveDrive::getRobotVelocity,
            (speedsRobotRelative, moduleFeedForwards) -> {
                swerveDrive.setChassisSpeeds(speedsRobotRelative);
            },
            new PPHolonomicDriveController(
                new PIDConstants(3.0, 0.0, 0.0),
                new PIDConstants(3.0, 0.0, 0.0)
            ),
            config,
            () -> false, // never flip for alliance
            this
        );
        System.out.println(">>> AutoBuilder.configure DONE");
    } catch (Exception e)
    {
        System.out.println(">>> AutoBuilder.configure FAILED");
        e.printStackTrace();
    }
}
   
   /**
   * Get the path follower with events.
   *
   * @param pathName PathPlanner path name.
   * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
   */
  public Command getAutonomousCommand(String pathName)
  {
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return new PathPlannerAuto(pathName);
  }






// in periodic:





@Override
public void periodic() {
    SmartDashboard.putNumber("Pose X", swerveDrive.getPose().getX());
    SmartDashboard.putNumber("Pose Y", swerveDrive.getPose().getY());
    SmartDashboard.putNumber("Pose Rotation", swerveDrive.getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("Velocity X", swerveDrive.getRobotVelocity().vxMetersPerSecond);
    SmartDashboard.putNumber("Velocity Y", swerveDrive.getRobotVelocity().vyMetersPerSecond);
    SmartDashboard.putNumber("IMU Yaw", swerveDrive.getYaw().getDegrees());
  
    SmartDashboard.putNumber("Actual Speed", 
    Math.hypot(
        swerveDrive.getRobotVelocity().vxMetersPerSecond,
        swerveDrive.getRobotVelocity().vyMetersPerSecond
    )
);


}


  
}