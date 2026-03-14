package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    // ── Top-level tuning variables ──────────────────────────────────────
    public static final double INTAKE_SPEED = .75; // 0.0 to 1.0
    
    // ───────────────────────────────────────────────────────────────────

    // CAN IDs – change to match your robot
    private static final int INTAKE_MOTOR_ID = 15;
  

    private final SparkMax intakeMotor;
   

    private boolean isRunning = false;

    @SuppressWarnings("removal")
    public IntakeSubsystem() {
        
        intakeMotor  = new SparkMax(INTAKE_MOTOR_ID,  MotorType.kBrushless);
        

        // Configure left motor
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig.idleMode(IdleMode.kCoast)
        .inverted(true);
        intakeMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

       
    }

    /** Toggle the INTAKE on/off at INTAKE_SPEED. */
    public void toggle() {
        if (isRunning) {
            stop();
        } else {
            start();
        }
    }

    public void start() {
        intakeMotor.set(INTAKE_SPEED);
    
        isRunning = true;
    }

    public void stop() {
        intakeMotor.set(0);
      
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