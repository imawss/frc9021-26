package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRollerSubsystem extends SubsystemBase {
    private final SparkMax rollerMotor;
    
    private static final int ROLLER_MOTOR_ID = 4;  

    private static final double INTAKE_SPEED = 0.8;  
    private static final double EJECT_SPEED = -0.6;   
    
    private double currentSpeed = 0.0;
    
    public IntakeRollerSubsystem() {
        rollerMotor = new SparkMax(ROLLER_MOTOR_ID, MotorType.kBrushless);
        
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast); 
        config.smartCurrentLimit(30);
        config.inverted(false);  
        
        rollerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("IntakeRoller/Speed", currentSpeed);
        SmartDashboard.putNumber("IntakeRoller/Current (A)", rollerMotor.getOutputCurrent());
        SmartDashboard.putBoolean("IntakeRoller/Is Running", isRunning());
    }

    public void intake() {
        setSpeed(INTAKE_SPEED);
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

    public boolean isRunning() {
        return Math.abs(currentSpeed) > 0.01;
    }
}