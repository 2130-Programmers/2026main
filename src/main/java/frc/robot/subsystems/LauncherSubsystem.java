package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LauncherSubsystem extends SubsystemBase {

    // ── Baseline (calibrated at BASELINE_DIST_FT) ───────────────────────
    public static final double BASELINE_DIST_FT  = 10.0;
    public static final double BASELINE_SPEED    = 0.55;
    public static final double BASELINE_RATIO    = 2.0;

    // ── Side balance correction ──────────────────────────────────────────
    // Right side is stronger — reduce it until both sides feel equal.
    // 1.0 = no correction, 0.9 = right runs at 90% of left
    public static final double RIGHT_TRIM = 0.95;

    // ── Speed scaling ────────────────────────────────────────────────────
    public static final double SPEED_PER_FOOT    = 0.01;
    public static final double MIN_SPEED         = 0.3;
    public static final double MAX_SPEED         = 1.0;

    // ── Ratio scaling ────────────────────────────────────────────────────
    public static final double RATIO_PER_FOOT    = 0.09;
    public static final double MIN_RATIO         = 1.0;
    public static final double MAX_RATIO         = 4.0;

    // ── Distance clamp ───────────────────────────────────────────────────
    public static final double MIN_DIST_FT       = 5.0;
    public static final double MAX_DIST_FT       = 25.0;

    // ── Close-range exponential boost ───────────────────────────────────
    // When below BASELINE_DIST_FT, apply an extra exponential pull-down
    // on speed and ratio so close shots don't overshoot.
    // Increase CLOSE_CURVE_STRENGTH to make the dip more aggressive.
    public static final double CLOSE_RANGE_THRESHOLD = BASELINE_DIST_FT; // feet
    public static final double CLOSE_CURVE_STRENGTH  = 0.04; // tune this (higher = more aggressive dip)
    // Overall scalar on the close-range correction: 0.0 = no correction, 1.0 = full, >1.0 = extra aggressive
    public static final double CLOSE_CORRECTION_SCALE = 0.17;

    private static final int TOP_MOTOR_ID     = 13;
    private static final int TOP_MOTOR_ID2    = 14;
    private static final int BOTTOM_MOTOR_ID  = 19;
    private static final int BOTTOM_MOTOR_ID2 = 18;

    private final SparkMax topMotor;
    private final SparkMax topMotor2;
    private final SparkMax bottomMotor;
    private final SparkMax bottomMotor2;

    private TargetLockSubsystem targetLock = null;

    private boolean isRunning   = false;
    private double  currentDist = BASELINE_DIST_FT;

    @SuppressWarnings("removal")
    public LauncherSubsystem() {
        topMotor     = new SparkMax(TOP_MOTOR_ID,     MotorType.kBrushless);
        topMotor2    = new SparkMax(TOP_MOTOR_ID2,    MotorType.kBrushless);
        bottomMotor  = new SparkMax(BOTTOM_MOTOR_ID,  MotorType.kBrushless);
        bottomMotor2 = new SparkMax(BOTTOM_MOTOR_ID2, MotorType.kBrushless);

        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig.idleMode(IdleMode.kCoast).inverted(true);
        topMotor.configure(leftConfig,  ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        topMotor2.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig.idleMode(IdleMode.kCoast).inverted(false);
        bottomMotor.configure(rightConfig,  ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        bottomMotor2.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setTargetLock(TargetLockSubsystem targetLock) {
        this.targetLock = targetLock;
    }

    public void setDistance(double distFeet) {
        currentDist = Math.max(MIN_DIST_FT, Math.min(MAX_DIST_FT, distFeet));
    }

    /**
     * Returns an exponential close-range correction factor (0.0 to 1.0).
     * At baseline distance: factor = 0 (no correction).
     * As distance shrinks below baseline: factor grows, pulling speed/ratio down.
     * Shape: exponential, so the effect is subtle mid-range but sharp at extremes.
     */
    private double closeRangeCorrectionFactor() {
        if (currentDist >= CLOSE_RANGE_THRESHOLD) return 0.0;

        // How far below the threshold are we? (positive value)
        double deficit = CLOSE_RANGE_THRESHOLD - currentDist;

        // Exponential growth: e^(k*deficit) - 1, normalized so deficit=5ft gives ~1.0
        double maxDeficit = CLOSE_RANGE_THRESHOLD - MIN_DIST_FT; // e.g. 5 ft
        double raw = Math.exp(CLOSE_CURVE_STRENGTH * deficit * (1.0 / maxDeficit) * 10.0) - 1.0;
        double maxRaw = Math.exp(CLOSE_CURVE_STRENGTH * 10.0) - 1.0;

        return Math.min(1.0, raw / maxRaw);
    }

    private double computeSpeed() {
        double delta = currentDist - BASELINE_DIST_FT;
        double linear = Math.max(MIN_SPEED, Math.min(MAX_SPEED,
                BASELINE_SPEED + (delta * SPEED_PER_FOOT)));

        // At close range, pull speed further down exponentially
        double correction = closeRangeCorrectionFactor();
        double corrected = linear - correction * (linear - MIN_SPEED) * 0.4 * CLOSE_CORRECTION_SCALE;

        return Math.max(MIN_SPEED, corrected);
    }

    private double computeRatio() {
        double delta = currentDist - BASELINE_DIST_FT;
        double linear = Math.max(MIN_RATIO, Math.min(MAX_RATIO,
                BASELINE_RATIO + (delta * RATIO_PER_FOOT)));

        // At close range, pull ratio further down exponentially
        double correction = closeRangeCorrectionFactor();
        double corrected = linear - correction * (linear - MIN_RATIO) * 0.4 * CLOSE_CORRECTION_SCALE;

        return Math.max(MIN_RATIO, corrected);
    }

    public void toggle() {
        if (isRunning) stop();
        else start();
    }

    public void start() { isRunning = true; }

    public void stop() {
        topMotor.set(0);
        topMotor2.set(0);
        bottomMotor.set(0);
        bottomMotor2.set(0);
        isRunning = false;
    }

    public boolean isRunning() { return isRunning; }

    @Override
    public void periodic() {
        if (targetLock != null && targetLock.hasTarget()) {
            setDistance(targetLock.getDistanceFeet());
        }

        double speed = computeSpeed();
        double ratio = computeRatio();

        if (isRunning) {
            // topMotor/topMotor2 = left side, bottomMotor/bottomMotor2 = right side
            topMotor.set(speed * -1 * RIGHT_TRIM);
            topMotor2.set(speed);
            bottomMotor.set(speed * ratio * RIGHT_TRIM * -1);
            bottomMotor2.set(speed * ratio);
        }
    }
}