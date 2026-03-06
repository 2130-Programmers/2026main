package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LauncherSubsystem extends SubsystemBase {

    // ── Top-level tuning variables ──────────────────────────────────────
    public static final double LAUNCHER_SPEED = .5;// 0.0 to 1.0
    public static final double LAUNCHER_Ratio = 2;//1:Launcher_Ratio
    // ───────────────────────────────────────────────────────────────────

    // CAN IDs – change to match your robot
    private static final int TOP_MOTOR_ID  = 13;
    private static final int TOP_MOTOR_ID2  = 14;
    private static final int BOTTOM_MOTOR_ID = 19;
    private static final int BOTTOM_MOTOR_ID2 = 18;


    private final SparkMax topMotor;
    private final SparkMax bottomMotor;


    private final SparkMax topMotor2;
    private final SparkMax bottomMotor2;

    private boolean isRunning = false;

    @SuppressWarnings("removal")
    public LauncherSubsystem() {
        topMotor  = new SparkMax(TOP_MOTOR_ID,  MotorType.kBrushless);
        bottomMotor = new SparkMax(BOTTOM_MOTOR_ID, MotorType.kBrushless);

        topMotor2  = new SparkMax(TOP_MOTOR_ID2,  MotorType.kBrushless);
        bottomMotor2 = new SparkMax(BOTTOM_MOTOR_ID2, MotorType.kBrushless);



        // Configure left motor
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig.idleMode(IdleMode.kCoast)
        .inverted(true);
        topMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        topMotor2.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // Configure right motor (inverted to run in same direction)
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig
            .idleMode(IdleMode.kCoast)
            .inverted(false);
        bottomMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        bottomMotor2.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /** Toggle the launcher on/off at LAUNCHER_SPEED. */
    public void toggle() {
        if (isRunning) {
            stop();
        } else {
            start();
        }
    }

    public void start() {
        topMotor.set(LAUNCHER_SPEED*-1);
        topMotor2.set(LAUNCHER_SPEED);

        bottomMotor.set(LAUNCHER_SPEED*LAUNCHER_Ratio*-1);
        bottomMotor2.set(LAUNCHER_SPEED*LAUNCHER_Ratio);
        isRunning = true;
    }

    public void stop() {
        topMotor.set(0);
        topMotor2.set(0);
        bottomMotor.set(0);
        bottomMotor2.set(0);
        isRunning = false;
    }

    public boolean isRunning() {
        return isRunning;
    }

    @Override
    public void periodic() {
        // Add SmartDashboard telemetry here if desired
    }
}