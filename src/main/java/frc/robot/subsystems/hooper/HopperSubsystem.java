package frc.robot.subsystems.hooper;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSubsystem extends SubsystemBase {
    private final SparkMax rollerMotor;
    
    private static final int ROLLER_MOTOR_ID = 5;  
    
    private static final double INTAKE_SPEED = 0.6;   
    private static final double FEED_SPEED = 0.7;     
    private static final double EJECT_SPEED = -0.5;   
    
    private double currentSpeed = 0.0;
    
    public HopperSubsystem() {
        rollerMotor = new SparkMax(ROLLER_MOTOR_ID, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(30);
        config.inverted(false);  
        
        rollerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        System.out.println("[Hopper] Initialized" );
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hopper/Speed", currentSpeed);
        SmartDashboard.putNumber("Hopper/Current (A)", rollerMotor.getOutputCurrent());
    }
    
    public void intake() {
        setSpeed(INTAKE_SPEED);
    }

    public void feed() {
        setSpeed(FEED_SPEED);
    }

    public void eject() {
        setSpeed(EJECT_SPEED);
    }

    public void stop() {
        setSpeed(0);
    }
    
    /**
     * Directly set roller speed.
     * @param speed -1.0 to 1.0
     */
    public void setSpeed(double speed) {
        currentSpeed = speed;
        rollerMotor.set(speed);
    }
    
    /**
     * Check if hopper is running.
     */
    public boolean isRunning() {
        return Math.abs(currentSpeed) > 0.01;
    }
}