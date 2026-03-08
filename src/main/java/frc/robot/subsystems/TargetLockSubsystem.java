package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;

/**
 * TargetLockSubsystem
 *
 * Tracks AprilTag IDs 26 and 10 only.
 * Rotates the robot to center the tag (yaw = 0). No offset logic.
 */
public class TargetLockSubsystem extends SubsystemBase {

    private final PhotonCamera camera;

    // Only lock onto these tag IDs
    private static final List<Integer> VALID_TAG_IDS = List.of(26, 10);

    // --- PID tuning ---
    private static final double kP = 0.04;
    private static final double kI = 0.0;
    private static final double kD = 0.001;

    private static final double TOLERANCE_DEG       = 1.5;
    private static final double MAX_ROT_RAD_PER_SEC = 3.0;

    private static final double CAMERA_RIGHT_IN = -2.7;
    private static final double DEPTH_BEHIND_IN = 23.5;

    private final PIDController rotController;

    private boolean locked        = false;
    private double  rotationSpeed = 0.0;
    private boolean hasTarget     = false;
    private double  distanceFeet  = 0.0;

    public TargetLockSubsystem(PhotonCamera camera) {
        this.camera = camera;

        rotController = new PIDController(kP, kI, kD);
        rotController.setTolerance(TOLERANCE_DEG);
        rotController.enableContinuousInput(-180.0, 180.0);
    }

    @Override
    public void periodic() {
        var result = camera.getLatestResult();

        PhotonTrackedTarget validTarget = null;
        if (result.hasTargets()) {
            for (PhotonTrackedTarget t : result.getTargets()) {
                if (VALID_TAG_IDS.contains(t.getFiducialId())) {
                    validTarget = t;
                    break;
                }
            }
        }

        hasTarget = (validTarget != null);

        if (hasTarget) {
            var transform = validTarget.getBestCameraToTarget();

            // Log all axes raw so we can see which one actually holds distance
        

            double rawDistM  = transform.getX();
            double rawDistIn = rawDistM * 39.3701;
            double rawDistFt = rawDistIn / 12.0;

            System.out.printf("[TargetLock] distance  raw=%.3fm  (%.1fin  %.2fft)  stored=%.2fft%n",
                    rawDistM, rawDistIn, rawDistFt, distanceFeet);

            distanceFeet = rawDistFt;
        }

        if (locked && hasTarget) {
            double yawError = validTarget.getYaw();

            double distIn = validTarget.getBestCameraToTarget().getX() * 39.3701;
            if (distIn < 1.0) distIn = 1.0;
            double totalDistIn = distIn + DEPTH_BEHIND_IN;
            double offsetDeg = Math.toDegrees(Math.atan2(CAMERA_RIGHT_IN, totalDistIn));

            double output = rotController.calculate(yawError, offsetDeg);

            output = Math.max(-1.0, Math.min(1.0, output)) * MAX_ROT_RAD_PER_SEC;
            rotationSpeed = rotController.atSetpoint() ? 0.0 : output;
        } else {
            rotationSpeed = 0.0;
            if (locked) rotController.reset();
        }
    }

    // -------------------------------------------------------------------------
    // Public API
    // -------------------------------------------------------------------------

    public void toggleLock() {
        locked = !locked;
        if (!locked) {
            rotController.reset();
            rotationSpeed = 0.0;
        }
    }

    public void enableLock()  { locked = true; }

    public void disableLock() {
        locked = false;
        rotController.reset();
        rotationSpeed = 0.0;
    }

    /** True only when lock is enabled AND a valid tag (26 or 10) is visible. */
    public boolean isLocked()      { return locked && hasTarget; }

    public boolean isLockEnabled() { return locked; }

    public boolean hasTarget()     { return hasTarget; }

    /** Distance to target in feet. Only valid when hasTarget() is true. */
    public double getDistanceFeet() { return distanceFeet; }

    /** Rotation correction in rad/s. Pass directly to swerve.drive() as rot. */
    public double getRotationSpeed() { return rotationSpeed; }
}