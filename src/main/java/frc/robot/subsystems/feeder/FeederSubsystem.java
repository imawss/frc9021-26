package frc.robot.subsystems.feeder;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Feeder subsystem - pushes game pieces from hopper up to shooter.
 * One NEO motor.
 * 
 * FUNCTIONS:
 * - Pull game piece from hopper
 * - Push game piece up into shooter wheels
 * - Control feed timing for shooting
 */
public class FeederSubsystem extends SubsystemBase {
    private final SparkMax feederMotor;
    
    private static final int FEEDER_MOTOR_ID = 36;   
    
    private static final double FEED_SPEED = 1;    
    private static final double SLOW_FEED_SPEED = 0.4; 
    private static final double REVERSE_SPEED = -0.5;  
    
    private double currentSpeed = 0.0;
    
    public FeederSubsystem() {
        feederMotor = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushless);
        
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast);
        config.smartCurrentLimit(30);
        config.inverted(true); 
        
        feederMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Feeder/Speed", currentSpeed);
        SmartDashboard.putNumber("Feeder/Current (A)", feederMotor.getOutputCurrent());

    }
    
    public void feed() {
        setSpeed(FEED_SPEED);
    }

    public void slowFeed() {
        setSpeed(SLOW_FEED_SPEED);
    }
 
    public void reverse() {
        setSpeed(REVERSE_SPEED);
    }

    public void stop() {
        setSpeed(0);
    }

    /**
     * Directly set feeder speed.
     * @param speed -1.0 to 1.0
     */
    public void setSpeed(double speed) {
        currentSpeed = speed;
        feederMotor.set(speed);
    }

    public boolean isRunning() {
        return Math.abs(currentSpeed) > 0.01;
    }
}