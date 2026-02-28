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
    // Positive = up, negative = down (from the left motor's perspective after inversion)
    public static final double PIVOT_SPEED             = 0.05;   // Speed going UP   — TUNE SIGN/MAGNITUDE
    public static final double DOWN_SPEED              = 0.15;   // Speed going DOWN  — TUNE SIGN/MAGNITUDE
    public static final double GEAR_RATIO              = 25.0;   // 25:1 gearbox
    public static final double TARGET_OUTPUT_ROTATIONS = 5.0;    // Output shaft rotations to reach top — TUNE THIS
    public static final double CURRENT_SPIKE_AMPS      = 25.0;   // Amps threshold for physical stop — TUNE THIS
    public static final double CURRENT_SETTLE_TIME     = 0.1;    // Seconds current must stay high before re-zero
    // ───────────────────────────────────────────────────────────────────

    private static final int LEFT_MOTOR_ID  = 16;
    private static final int RIGHT_MOTOR_ID = 17;

    private static final double PERIODIC_DT = 0.02; // 20ms loop

    private final SparkMax leftMotor;
    private final SparkMax rightMotor;
    private final RelativeEncoder encoder;

    private double zeroPositionMotorRotations = 0.0;
    private boolean isZeroed = false;

    public enum PivotState {
        AT_BOTTOM,   // Resting at physical stop / zeroed
        MOVING_UP,   // Driving toward top position
        AT_TOP,      // Holding at top (brake mode)
        ZEROING      // Pressing against bottom stop, watching for current spike
    }

    private PivotState currentState = PivotState.AT_BOTTOM;
    private double currentSpikeTimer = 0.0;

    public IntakePivotSubsystem() {
        leftMotor  = new SparkMax(LEFT_MOTOR_ID,  MotorType.kBrushless);
        rightMotor = new SparkMax(RIGHT_MOTOR_ID, MotorType.kBrushless);

        encoder = leftMotor.getEncoder();

        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig.idleMode(IdleMode.kBrake).inverted(true);
        leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig.idleMode(IdleMode.kBrake).inverted(false);
        rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        currentState = PivotState.AT_BOTTOM;
        isZeroed = false;
    }

    // ── Public API ──────────────────────────────────────────────────────

    /**
     * Toggle between UP and DOWN positions.
     * Ignores calls while already moving.
     */
    public void toggle() {
        switch (currentState) {
            case AT_BOTTOM:
                if (isZeroed) {
                    currentState = PivotState.MOVING_UP;
                } else {
                    currentState = PivotState.ZEROING;
                    currentSpikeTimer = 0.0;
                }
                break;

            case AT_TOP:
                // Drive back down and re-zero
                currentState = PivotState.ZEROING;
                currentSpikeTimer = 0.0;
                break;

            case MOVING_UP:
            case ZEROING:
                // Ignore while in motion
                break;
        }
    }

    /** Manually trigger zeroing sequence — call on enable or after brownout. */
    public void startZeroSequence() {
        currentState = PivotState.ZEROING;
        currentSpikeTimer = 0.0;
    }

    public PivotState getState()          { return currentState; }
    public boolean isZeroed()             { return isZeroed; }
    public double getMotorRotations()     { return encoder.getPosition(); }

    /** Output shaft rotations traveled upward from zero (bottom). */
    public double getOutputRotations() {
        return (encoder.getPosition() - zeroPositionMotorRotations) / GEAR_RATIO;
    }

    // ── Periodic State Machine ──────────────────────────────────────────

    @Override
    public void periodic() {
        double outputRotations = getOutputRotations();
        double leftAmps  = leftMotor.getOutputCurrent();
        double rightAmps = rightMotor.getOutputCurrent();
        double avgAmps   = (leftAmps + rightAmps) / 2.0;

        switch (currentState) {

            case AT_BOTTOM:
                stopMotors();
                break;

            case MOVING_UP:
                if (outputRotations >= TARGET_OUTPUT_ROTATIONS) {
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
                // Drive downward (negative direction)
                setMotors(-DOWN_SPEED);

                // Accumulate time current stays above threshold
                if (avgAmps >= CURRENT_SPIKE_AMPS) {
                    currentSpikeTimer += PERIODIC_DT;
                }
                // Note: we do NOT reset the timer on a dip — brushless motors
                // can briefly drop current during a stall. Once it starts rising
                // we just wait for CURRENT_SETTLE_TIME total accumulation.

                if (currentSpikeTimer >= CURRENT_SETTLE_TIME) {
                    stopMotors();
                    zeroPositionMotorRotations = encoder.getPosition();
                    isZeroed = true;
                    currentSpikeTimer = 0.0;
                    currentState = PivotState.AT_BOTTOM;
                    System.out.println("[IntakePivot] Zeroed at motor position: " + zeroPositionMotorRotations);
                }
                break;
        }

        // ── SmartDashboard Telemetry ──
        SmartDashboard.putString("IntakePivot/State",           currentState.toString());
        SmartDashboard.putNumber("IntakePivot/OutputRotations", outputRotations);
        SmartDashboard.putNumber("IntakePivot/MotorRotations",  encoder.getPosition());
        SmartDashboard.putNumber("IntakePivot/LeftAmps",        leftAmps);
        SmartDashboard.putNumber("IntakePivot/RightAmps",       rightAmps);
        SmartDashboard.putNumber("IntakePivot/SpikeTimer",      currentSpikeTimer);
        SmartDashboard.putBoolean("IntakePivot/IsZeroed",       isZeroed);
        SmartDashboard.putNumber("IntakePivot/ZeroPoint",       zeroPositionMotorRotations);
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