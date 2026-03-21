package frc.robot.subsystems;

import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AgitatorSubsystem extends SubsystemBase {

    // ── Top-level tuning variables ──────────────────────────────────────
    public static final double Agitator_SPEED = .150; // 0.0 to 1.0
    
    // ───────────────────────────────────────────────────────────────────

    // CAN IDs – change to match your robot
    private static final int Agitator_MOTOR_ID = 20;
    private static final int Agitator2_MOTOR_ID = 21;
    

    private final SparkMax agitatorMotor;
   
     private final SparkMax agitator2Motor;

    private boolean isRunning = false;

    @SuppressWarnings("removal")
    public AgitatorSubsystem() {
        
        agitatorMotor  = new SparkMax(Agitator_MOTOR_ID,  MotorType.kBrushless);
        agitator2Motor  = new SparkMax(Agitator2_MOTOR_ID,  MotorType.kBrushless);

        // Configure left motor
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig.idleMode(IdleMode.kCoast)
        .inverted(false).voltageCompensation(12);
        agitatorMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
       
        // Configure left motor
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig.idleMode(IdleMode.kCoast)
        .inverted(true).voltageCompensation(12);
        agitator2Motor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

       
    }

    /** Toggle the INTAKE on/off at Agitator_SPEED. */
    public void toggle() {
        if (isRunning) {
            stop();
        } else {
            start();
        }
    }

    public void start() {
        agitatorMotor.set(Agitator_SPEED);
        agitator2Motor.set(Agitator_SPEED);
        isRunning = true;
    }

    public void stop() {
        agitatorMotor.set(0);
        agitator2Motor.set(0);
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