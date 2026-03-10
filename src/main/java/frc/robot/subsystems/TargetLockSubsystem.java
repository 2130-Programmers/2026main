package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;

/**
 * TargetLockSubsystem
 *
 * Aims robot center at a point TARGET_DEPTH_IN inches BEHIND the AprilTag,
 * projected perpendicular to the wall using the tag's own facing direction.
 *
 * KEY FIXES:
 *   1. Camera offset applied to tag POSITION only — not to the wall normal.
 *   2. Wall normal derived from tag rotation MATRIX column 0, not Euler angles.
 *      getX/Y/Z() on Rotation3d returns roll/pitch/yaw — not axis direction vectors.
 *      toMatrix().get(0, 0) / get(1, 0) gives the actual tag X-axis direction in
 *      camera frame, which is the true perpendicular into the wall.
 *
 * CORRECT ORDER:
 *   1. Get tag position in camera frame (meters → inches)
 *   2. Shift to robot center frame (apply camera offsets to position only)
 *   3. Extract wall normal from rotation matrix column 0 (tag X-axis = into wall)
 *   4. Normalize the wall normal
 *   5. Project aim point: tag position + normal × TARGET_DEPTH_IN
 *   6. atan2(aimY, aimX) → heading error
 *   7. PID drives heading error → 0
 *
 * COORDINATE SYSTEM (WPILib 2026 / modern PhotonVision):
 *   +X = forward, +Y = left, +Z = up
 *
 * CAMERA_LATERAL_IN : positive = camera RIGHT of robot center, negative = LEFT
 * CAMERA_FORWARD_IN : positive = camera FORWARD of robot center, negative = BEHIND
 *
 * Camera is assumed to be aligned with robot front (no yaw rotation).
 */
public class TargetLockSubsystem extends SubsystemBase {

    // ── Valid AprilTag IDs ────────────────────────────────────────────────────
    private static final List<Integer> VALID_TAG_IDS = List.of(26, 10,2,8,21,18);

    // ── PID ───────────────────────────────────────────────────────────────────
    private static final double kP             = 0.04;
    private static final double kI             = 0.0;
    private static final double kD             = 0.001;
    private static final double TOLERANCE_DEG  = 1.5;
    private static final double MAX_ROT_OUTPUT = 3.0;   // rad/s cap

    // ── Geometry ──────────────────────────────────────────────────────────────
    /** Inches behind the tag face to aim at. Always positive. */
    private static final double TARGET_DEPTH_IN   = -23.5;

    /** Camera lateral offset from robot center. RIGHT = positive, LEFT = negative. */
    private static final double CAMERA_LATERAL_IN = 0;

    /** Camera forward offset from robot center. FORWARD = positive, BEHIND = negative. */
    private static final double CAMERA_FORWARD_IN = 0.0;

    // ── Output tuning ─────────────────────────────────────────────────────────
    private static final boolean INVERT_OUTPUT    = true;
    private static final boolean ENABLE_SLEW_RATE = true;
    private static final double  SLEW_RATE_LIMIT  = 6.0;
    private static final boolean ENABLE_DASHBOARD = true;

    // =========================================================================

    private final PhotonCamera    camera;
    private final PIDController   rotController;
    private final SlewRateLimiter slewLimiter;

    private boolean locked        = false;
    private double  rotationSpeed = 0.0;
    private boolean hasTarget     = false;

    // Dashboard telemetry
    private double dbCameraYaw    = 0.0;
    private double dbHeadingError = 0.0;
    private double dbDistanceFeet = 0.0;
    private double dbAimX         = 0.0;
    private double dbAimY         = 0.0;
    private double dbWallNx       = 0.0;
    private double dbWallNy       = 0.0;

    public TargetLockSubsystem(PhotonCamera camera) {
        this.camera = camera;
        rotController = new PIDController(kP, kI, kD);
        rotController.setTolerance(TOLERANCE_DEG);
        slewLimiter = new SlewRateLimiter(SLEW_RATE_LIMIT);
    }

    @Override
    public void periodic() {
        var result = camera.getLatestResult();

        // ── Find best valid target ────────────────────────────────────────────
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

        if (locked && hasTarget) {
            double cameraYaw = validTarget.getYaw();

            // ── Step 1: Tag position in camera frame (inches) ─────────────────
            // Modern PhotonVision (2026): +X = forward, +Y = left, +Z = up
            Transform3d camToTag = validTarget.getBestCameraToTarget();
            double tagX_cam = camToTag.getTranslation().getX() * 39.3701;
            double tagY_cam = camToTag.getTranslation().getY() * 39.3701;

            // ── Step 2: Shift to robot center frame ───────────────────────────
            // Apply camera offsets to tag POSITION only.
            // The wall normal is derived from the tag's rotation and must NOT
            // have positional offsets mixed in.
            double tagX_robot = tagX_cam + CAMERA_FORWARD_IN;
            double tagY_robot = tagY_cam - CAMERA_LATERAL_IN;

            // ── Step 3: Wall normal from tag rotation matrix ──────────────────
            // CRITICAL: Rotation3d.getX/Y/Z() returns Euler angles (roll/pitch/yaw),
            // NOT direction vectors of the tag's axes. Using those as a direction
            // vector produces a wrong normal, causing the robot to turn away when
            // any camera offset is present.
            //
            // toMatrix() column 0 = where the tag's +X axis points in camera frame.
            // In PhotonVision the tag +X axis points INTO the wall (away from camera),
            // giving the true wall normal regardless of robot approach angle.
            var rotMatrix = camToTag.getRotation().toMatrix();
            double rx = rotMatrix.get(0, 0); // tag X projected onto camera/robot forward
            double ry = rotMatrix.get(1, 0); // tag X projected onto camera/robot left

            double nLen = Math.hypot(rx, ry);
            if (nLen < 0.001) {
                // Fallback: degenerate rotation (tag nearly parallel to view axis),
                // use robot-to-tag vector as best approximation.
                double dist2D = Math.hypot(tagX_robot, tagY_robot);
                if (dist2D < 1.0) {
                    rx = 1.0; ry = 0.0;
                } else {
                    rx = tagX_robot / dist2D;
                    ry = tagY_robot / dist2D;
                }
            } else {
                rx /= nLen;
                ry /= nLen;
            }

            // ── Step 4: Aim point behind the tag, perpendicular to wall ───────
            // Tag position + wall normal × depth = point TARGET_DEPTH_IN inches
            // directly behind the tag face into the wall.
            // Because the normal comes from tag rotation, this stays fixed on the
            // wall regardless of what angle the robot is approaching from.
            double aimX = tagX_robot + rx * TARGET_DEPTH_IN;
            double aimY = tagY_robot + ry * TARGET_DEPTH_IN;

            // ── Step 5: Heading error ─────────────────────────────────────────
            double headingError = Math.toDegrees(Math.atan2(aimY, aimX));

            // ── Step 6: PID → rotation output ────────────────────────────────
            double output = rotController.calculate(headingError, 0.0);
            output = Math.max(-1.0, Math.min(1.0, output)) * MAX_ROT_OUTPUT;
            if (INVERT_OUTPUT)    output = -output;
            if (ENABLE_SLEW_RATE) output = slewLimiter.calculate(output);

            rotationSpeed  = rotController.atSetpoint() ? 0.0 : output;
            dbCameraYaw    = cameraYaw;
            dbHeadingError = headingError;
            dbDistanceFeet = tagX_robot / 12.0;
            dbAimX         = aimX;
            dbAimY         = aimY;
            dbWallNx       = rx;
            dbWallNy       = ry;

        } else {
            rotationSpeed = ENABLE_SLEW_RATE ? slewLimiter.calculate(0.0) : 0.0;
            if (locked) rotController.reset();
        }

        if (ENABLE_DASHBOARD) {
            SmartDashboard.putBoolean("Locked",          locked);
            SmartDashboard.putBoolean("HasTarget",        hasTarget);
            SmartDashboard.putBoolean("AtSetpoint",       rotController.atSetpoint());
            SmartDashboard.putNumber ("CameraYaw_deg",    dbCameraYaw);
            SmartDashboard.putNumber ("HeadingError_deg", dbHeadingError);
            SmartDashboard.putNumber ("Distance_ft",      dbDistanceFeet);
            SmartDashboard.putNumber ("AimX_in",          dbAimX);
            SmartDashboard.putNumber ("AimY_in",          dbAimY);
            SmartDashboard.putNumber ("WallNormal_X",     dbWallNx);
            SmartDashboard.putNumber ("WallNormal_Y",     dbWallNy);
            SmartDashboard.putNumber ("Output_radps",     rotationSpeed);
        }
    }

    // ── Lock control ──────────────────────────────────────────────────────────

    public void toggleLock() {
        locked = !locked;
        if (!locked) reset();
    }

    public void enableLock()  { locked = true; }

    public void disableLock() { locked = false; reset(); }

    private void reset() {
        rotController.reset();
        slewLimiter.reset(0.0);
        rotationSpeed = 0.0;
    }

    // ── Getters ───────────────────────────────────────────────────────────────

    public boolean isLocked()         { return locked && hasTarget; }
    public boolean isLockEnabled()    { return locked; }
    public boolean hasTarget()        { return hasTarget; }
    public double  getDistanceFeet()  { return dbDistanceFeet; }
    public double  getRotationSpeed() { return rotationSpeed; }
}