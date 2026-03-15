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
    public static final double PIVOT_SPEED      = 0.22;
    public static final double MANUAL_SPEED     = 0.3; // TUNE THIS
    public static final double GEAR_RATIO       = 25.0;

    public static final double STABLE_THRESHOLD = 0.01;
    public static final double STABLE_TIME      = 0.15;

    public static final double PUSH_SPEED       = 0.1;
    public static final double PUSH_DURATION    = 0.9;
    // ───────────────────────────────────────────────────────────────────

    private static final int LEFT_MOTOR_ID  = 16;
    private static final int RIGHT_MOTOR_ID = 17;

    private static final double PERIODIC_DT = 0.02;

    private final SparkMax leftMotor;
    private final SparkMax rightMotor;
    private final RelativeEncoder encoder;

    private double targetDegrees;
    private double targetOutputRotations;

    private double zeroPositionMotorRotations = 0.0;
    private boolean isZeroed = false;

    private double lastEncoderPosition = 0.0;
    private double stableTimer         = 0.0;
    private double pushTimer           = 0.0;

    private boolean returnToTopAfterZero = true;

    public enum PivotState {
        AT_BOTTOM,
        MOVING_UP,
        AT_TOP,
        ZEROING,
        MANUAL  // ← new
    }

    private PivotState currentState  = PivotState.AT_TOP;
    private PivotState stateBeforeManual = PivotState.AT_TOP; // restore after manual

    @SuppressWarnings("removal")
    public IntakePivotSubsystem(double targetDegrees) {
        this.targetDegrees         = targetDegrees;
        this.targetOutputRotations = targetDegrees / 360.0;

        leftMotor  = new SparkMax(LEFT_MOTOR_ID,  MotorType.kBrushless);
        rightMotor = new SparkMax(RIGHT_MOTOR_ID, MotorType.kBrushless);

        encoder = leftMotor.getEncoder();

        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig.idleMode(IdleMode.kBrake).inverted(true);
        leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig.idleMode(IdleMode.kBrake).inverted(false);
        rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // ── Public API ──────────────────────────────────────────────────────

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
            case MANUAL:
                break;
        }
    }

    public void startZeroSequence() {
        stableTimer         = 0.0;
        pushTimer           = 0.0;
        lastEncoderPosition = encoder.getPosition();
        currentState        = PivotState.ZEROING;
    }

    public void setTargetDegrees(double degrees) {
        this.targetDegrees         = degrees;
        this.targetOutputRotations = degrees / 360.0;
    }

   /** Call while d-pad right is held. */
 /** Call while d-pad right is held. */
    public void manualForward() {
        if (currentState != PivotState.MANUAL) {
            stateBeforeManual = currentState;  // only save on first entry
            currentState      = PivotState.MANUAL;
        }
        if (getOutputRotations() * 360.0 < 93.0) {
            setMotors(MANUAL_SPEED);
        } else {
            stopMotors();
        }
    }

    /** Call while d-pad left is held. */
    public void manualReverse() {
        if (currentState != PivotState.MANUAL) {
            stateBeforeManual = currentState;  // only save on first entry
            currentState      = PivotState.MANUAL;
        }
        if (getOutputRotations() * 360.0 > 0.0) {
            setMotors(-MANUAL_SPEED);
        } else {
            stopMotors();
        }
    }
    /** Call on d-pad release — stops motors and restores previous state. */
    public void manualStop() {
        stopMotors();
        currentState = stateBeforeManual;
    }

    public PivotState getState()      { return currentState; }
    public boolean isZeroed()         { return isZeroed; }
    public double getMotorRotations() { return encoder.getPosition(); }
    public double getTargetDegrees()  { return targetDegrees; }

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
                if (Math.abs(outputRotations) >= targetOutputRotations) {
                    stopMotors();
                    currentState = PivotState.AT_TOP;
                } else {
                    setMotors(PIVOT_SPEED);
                }
                break;

            case AT_TOP:
                stopMotors();
                break;

            case MANUAL:
                // Motors driven directly by manualForward/manualReverse — nothing to do here
                break;

            case ZEROING:
                if (pushTimer < PUSH_DURATION) {
                    setMotors(-PUSH_SPEED);
                    pushTimer += PERIODIC_DT;
                    break;
                }

                stopMotors();

                double delta = Math.abs(currentPosition - lastEncoderPosition);
                lastEncoderPosition = currentPosition;

                if (delta < STABLE_THRESHOLD) {
                    stableTimer += PERIODIC_DT;
                } else {
                    stableTimer = 0.0;
                }

                if (stableTimer >= STABLE_TIME) {
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

        SmartDashboard.putString ("IntakePivot/State",           currentState.toString());
        SmartDashboard.putNumber ("IntakePivot/OutputRotations", outputRotations);
        SmartDashboard.putNumber ("IntakePivot/MotorRotations",  currentPosition);
        SmartDashboard.putNumber ("IntakePivot/StableTimer",     stableTimer);
        SmartDashboard.putNumber ("IntakePivot/PushTimer",       pushTimer);
        SmartDashboard.putBoolean("IntakePivot/IsZeroed",        isZeroed);
        SmartDashboard.putNumber ("IntakePivot/ZeroPoint",       zeroPositionMotorRotations);
        SmartDashboard.putNumber ("IntakePivot/TargetDegrees",   targetDegrees);
        SmartDashboard.putNumber ("IntakePivot/TargetRotations", targetOutputRotations);
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