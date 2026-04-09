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
    public static final double PIVOT_SPEED      = 0.2;
    public static final double MANUAL_SPEED     = 0.2; // TUNE THIS
    public static final double GEAR_RATIO       = 25.0;

    public static final double STABLE_THRESHOLD = 0.01;
    public static final double STABLE_TIME      = 0.15;

    public static final double PUSH_SPEED       = 0.1;
    public static final double PUSH_DURATION    = 0.9;

    // How hard to actively hold the pivot down against ball pressure.
    // Negative = pushing down. Tune until it resists balls but doesn't
    // slam the hard stop. Start low (~0.05) and increase if it still drifts.
    public static final double HOLD_DOWN_SPEED  = 0.08; // TUNE THIS

    // ── Agitation Tuning ────────────────────────────────────────────────
    public static final double AGITATE_SPEED    = 0.3;  // TUNE THIS
    public static final double AGITATE_LOW_DEG  = 0.0;  // lower bound (degrees)
    public static final double AGITATE_HIGH_DEG = 60.0; // upper bound (degrees)
    public static final double AGITATE_DEADBAND = 1.0;  // degrees of tolerance at each end
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

    // Tracks agitation direction: true = moving toward HIGH, false = moving toward LOW
    private boolean agitatingUp = true;

    // Sole source of truth for hold-down. Only ever flipped by toggleHoldDown().
    private boolean holdDownActive = false;

    public enum PivotState {
        AT_BOTTOM,
        MOVING_UP,
        AT_TOP,
        ZEROING,
        AGITATING,
        MANUAL
    }

    private PivotState currentState      = PivotState.AT_TOP;
    private PivotState stateBeforeManual = PivotState.AT_TOP;

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
            case AGITATING:
            case MANUAL:
                break;
        }
    }

    /**
     * Begin oscillating the pivot between AGITATE_LOW_DEG and AGITATE_HIGH_DEG.
     */
    public void startAgitate() {
        double currentDeg = getOutputRotations() * 360.0;
        agitatingUp = (currentDeg < (AGITATE_LOW_DEG + AGITATE_HIGH_DEG) / 2.0);
        currentState = PivotState.AGITATING;
        System.out.println("[IntakePivot] Starting agitation. Initial direction: "
                + (agitatingUp ? "UP" : "DOWN"));
    }

    /**
     * Stop agitation and rest at AT_BOTTOM.
     */
    public void stopAgitate() {
        stopMotors();
        currentState = PivotState.AT_BOTTOM;
        System.out.println("[IntakePivot] Agitation stopped.");
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
    public void manualForward() {
        if (currentState != PivotState.MANUAL) {
            stateBeforeManual = currentState;
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
            stateBeforeManual = currentState;
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

    /**
     * Toggles active hold-down on/off. This is the ONLY way hold-down turns on.
     * Bind in RobotContainer with NO subsystem requirement to avoid conflicts:
     *   button.onTrue(new InstantCommand(pivot::toggleHoldDown))
     */
    public void toggleHoldDown() {
        holdDownActive = !holdDownActive;
        System.out.println("[IntakePivot] HoldDown: " + (holdDownActive ? "ON" : "OFF"));
    }

    public boolean isHoldDownActive() { return holdDownActive; }
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
        double currentDeg      = outputRotations * 360.0;

        switch (currentState) {

            case AT_BOTTOM:
                // Hold-down is entirely controlled by toggleHoldDown().
                // When active, resist ball pressure. When inactive, just brake.
                if (holdDownActive && currentDeg > 1.0) {
                    setMotors(-HOLD_DOWN_SPEED);
                } else {
                    stopMotors();
                }
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

            case AGITATING:
                if (agitatingUp) {
                    if (currentDeg >= AGITATE_HIGH_DEG - AGITATE_DEADBAND) {
                        agitatingUp = false;
                        System.out.println("[IntakePivot] Agitate: reversing DOWN at " + currentDeg + "°");
                    } else {
                        setMotors(AGITATE_SPEED);
                    }
                } else {
                    if (currentDeg <= AGITATE_LOW_DEG + AGITATE_DEADBAND) {
                        agitatingUp = true;
                        System.out.println("[IntakePivot] Agitate: reversing UP at " + currentDeg + "°");
                    } else {
                        setMotors(-AGITATE_SPEED);
                    }
                }
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
        SmartDashboard.putBoolean("IntakePivot/AgitatingUp",     agitatingUp);
        SmartDashboard.putNumber ("IntakePivot/CurrentDeg",      currentDeg);
        SmartDashboard.putBoolean("IntakePivot/HoldDownActive",  holdDownActive);
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

    public void resetToTop() {
        currentState = PivotState.AT_TOP;
        returnToTopAfterZero = true;
        stableTimer = 0.0;
        pushTimer = 0.0;
    }
}