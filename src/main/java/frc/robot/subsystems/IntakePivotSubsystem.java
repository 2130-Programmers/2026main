package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePivotSubsystem extends SubsystemBase {

    // ── Tuning Variables ────────────────────────────────────────────────
    public static final double PIVOT_SPEED             = 0.15;          // Speed going UP               — TUNE SIGN/MAGNITUDE
    public static final double GEAR_RATIO              = 25.0;          // 25:1 gearbox

    // 60 degrees = 60/360 = 0.1667 output shaft rotations
    public static final double TARGET_OUTPUT_ROTATIONS = 93.0 / 360.0; // ~0.1667 output shaft rotations for 60°

    // Encoder stabilization for gravity-drop zeroing:
    // If the encoder delta stays below STABLE_THRESHOLD for STABLE_TIME seconds, we've settled at bottom.
    public static final double STABLE_THRESHOLD        = 0.01;          // Motor rotations per loop considered "not moving" — TUNE THIS
    public static final double STABLE_TIME             = 0.15;          // Seconds of stability required to confirm bottom  — TUNE THIS
    // ───────────────────────────────────────────────────────────────────

    private static final int LEFT_MOTOR_ID  = 16;
    private static final int RIGHT_MOTOR_ID = 17;

    private static final double PERIODIC_DT = 0.02; // 20ms loop

    private final SparkMax leftMotor;
    private final SparkMax rightMotor;
    private final RelativeEncoder encoder;

    private double zeroPositionMotorRotations = 0.0;
    private boolean isZeroed = false;

    // Gravity-drop stabilization tracking
    private double lastEncoderPosition = 0.0;
    private double stableTimer         = 0.0;

    // Brief push to break balance before releasing to gravity
    public static final double PUSH_SPEED    = 0.1;  // Downward nudge power — TUNE THIS
    public static final double PUSH_DURATION = 0.9; // Seconds to push      — TUNE THIS
    private double pushTimer = 0.0;

    // After startup zero, return to top
    private boolean returnToTopAfterZero = true;

    public enum PivotState {
        AT_BOTTOM,  // Resting at physical bottom stop, zeroed
        MOVING_UP,  // Driving up toward 60° position
        AT_TOP,     // Holding at top (brake mode)
        ZEROING     // Motors released — falling via gravity, watching encoder to stabilize
    }

    // Robot assumed to start in UP position
    private PivotState currentState = PivotState.AT_TOP;

    @SuppressWarnings("removal")
    public IntakePivotSubsystem() {
        leftMotor  = new SparkMax(LEFT_MOTOR_ID,  MotorType.kBrushless);
        rightMotor = new SparkMax(RIGHT_MOTOR_ID, MotorType.kBrushless);

        encoder = leftMotor.getEncoder();

        // Motors inverted opposite to each other (flipped from before)
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig.idleMode(IdleMode.kBrake).inverted(true);
        leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig.idleMode(IdleMode.kBrake).inverted(false);
        rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // ── Public API ──────────────────────────────────────────────────────

    /**
     * Toggle between UP and DOWN positions.
     * Ignores calls while already moving or zeroing.
     *
     *   AT_TOP    → release motors, fall by gravity, zero when encoder stabilizes → AT_BOTTOM
     *   AT_BOTTOM → drive up 60° → AT_TOP
     */
    public void toggle() {
        switch (currentState) {

            case AT_TOP:
                returnToTopAfterZero = false;
                startZeroSequence();
                break;

            case AT_BOTTOM:
                currentState = PivotState.MOVING_UP;
                break;

            case MOVING_UP:
            case ZEROING:
                // Ignore while in motion
                break;
        }
    }

    /**
     * Release motors and let gravity drop the pivot to bottom.
     * Encoder stabilization detects when it has settled.
     * Call on enable or after brownout — will return to top afterward.
     */
    public void startZeroSequence() {
        stableTimer         = 0.0;
        pushTimer           = 0.0;
        lastEncoderPosition = encoder.getPosition();
        currentState        = PivotState.ZEROING;
    }

    public PivotState getState()      { return currentState; }
    public boolean isZeroed()         { return isZeroed; }
    public double getMotorRotations() { return encoder.getPosition(); }

    /** Output shaft rotations traveled upward from zero (bottom). */
    public double getOutputRotations() {
        return (encoder.getPosition() - zeroPositionMotorRotations) / GEAR_RATIO;
    }

    // ── Periodic State Machine ──────────────────────────────────────────

    @Override
    public void periodic() {
        double outputRotations = getOutputRotations();
        double currentPosition = encoder.getPosition();

        switch (currentState) {

            case AT_BOTTOM:
                stopMotors();
                break;

            case MOVING_UP:
                if (Math.abs(outputRotations) >= TARGET_OUTPUT_ROTATIONS) {
                    stopMotors();
                    currentState = PivotState.AT_TOP;
                } else {
                    setMotors(PIVOT_SPEED); // positive = up
                }
                break;

            case AT_TOP:
                stopMotors(); // brake mode holds position passively
                break;

            case ZEROING:
                // Phase 1: brief downward push to break balance point
                if (pushTimer < PUSH_DURATION) {
                    setMotors(-PUSH_SPEED);
                    pushTimer += PERIODIC_DT;
                    break; // skip stabilization check until push is done
                }

                // Phase 2: release to gravity — brake mode slows the fall
                // and holds firmly once it reaches the physical stop.
                stopMotors();

                // Measure how much the encoder moved since last loop
                double delta = Math.abs(currentPosition - lastEncoderPosition);
                lastEncoderPosition = currentPosition;

                if (delta < STABLE_THRESHOLD) {
                    stableTimer += PERIODIC_DT; // encoder barely moving — accumulate stable time
                } else {
                    stableTimer = 0.0;          // still falling — reset
                }

                if (stableTimer >= STABLE_TIME) {
                    // Encoder stable long enough — we're at the bottom
                    zeroPositionMotorRotations = currentPosition;
                    isZeroed    = true;
                    stableTimer = 0.0;
                    System.out.println("[IntakePivot] Zeroed at motor position: " + zeroPositionMotorRotations);

                    if (returnToTopAfterZero) {
                        returnToTopAfterZero = false;
                        currentState = PivotState.MOVING_UP;
                        System.out.println("[IntakePivot] Returning to top after startup zero.");
                    } else {
                        currentState = PivotState.AT_BOTTOM;
                    }
                }
                break;
        }

        // ── SmartDashboard Telemetry ──
        SmartDashboard.putString ("IntakePivot/State",           currentState.toString());
        SmartDashboard.putNumber ("IntakePivot/OutputRotations", outputRotations);
        SmartDashboard.putNumber ("IntakePivot/MotorRotations",  currentPosition);
        SmartDashboard.putNumber ("IntakePivot/StableTimer",     stableTimer);
        SmartDashboard.putNumber ("IntakePivot/PushTimer",       pushTimer);
        SmartDashboard.putBoolean("IntakePivot/IsZeroed",        isZeroed);
        SmartDashboard.putNumber ("IntakePivot/ZeroPoint",       zeroPositionMotorRotations);
        SmartDashboard.putNumber ("IntakePivot/TargetRotations", TARGET_OUTPUT_ROTATIONS);
    }

    // ── Private Helpers ─────────────────────────────────────────────────

    private void setMotors(double speed) {
        leftMotor.set(speed);
        rightMotor.set(speed);
    }

    private void stopMotors() {
        leftMotor.set(0);
        rightMotor.set(0);
    }
}