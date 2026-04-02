package frc.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

    private static final int    MOTOR_CAN_ID   = 39;
    private static final double GEAR_RATIO      = 80.0;        
    private static final boolean MOTOR_INVERTED = false;       

    private static final double DEG_PER_MOTOR_ROT  = 360.0 / GEAR_RATIO;   
    private static final double DEG_PER_SEC_FACTOR = DEG_PER_MOTOR_ROT / 60.0; 

    private static final double MIN_ANGLE_DEG = 0.0;    
    private static final double MAX_ANGLE_DEG = 124;  

    private static final double SOFT_LIMIT_ZONE_DEG = 10.0;

    private static final double MANUAL_SPEED      = 0.25;  
    private static final double MANUAL_SPEED_SLOW = 0.05;  
    
    private static final double kG = 0.04;  

    private static final int CURRENT_LIMIT_AMPS = 40;

    private static final double HOMING_SPEED          = -0.06; 
    private static final double STALL_VELOCITY_THRESH = 0.5;   
    private static final double STALL_TIME_THRESH     = 0.25;  
    private static final double HOMING_IGNORE_TIME    = 0.4;   

    private static final boolean USE_LIMIT_SWITCH   = false;   
    private static final int     LIMIT_SWITCH_DIO   = 0;       

    private final SparkMax        motor;
    private final RelativeEncoder encoder;
    private final DigitalInput    limitSwitch;      

    private boolean isHomed     = false;
    private final Timer stallTimer = new Timer();

    public ArmSubsystem() {
        motor   = new SparkMax(MOTOR_CAN_ID, MotorType.kBrushless);
        encoder = motor.getEncoder();

        limitSwitch = USE_LIMIT_SWITCH ? new DigitalInput(LIMIT_SWITCH_DIO) : null;

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);              
        config.smartCurrentLimit(CURRENT_LIMIT_AMPS);
        config.inverted(MOTOR_INVERTED);

        config.encoder.positionConversionFactor(DEG_PER_MOTOR_ROT);
        config.encoder.velocityConversionFactor(DEG_PER_SEC_FACTOR);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder.setPosition(0.0);
    }

    public double getAngleDeg() {
        return encoder.getPosition();
    }

    public double getVelocityDegPerSec() {
        return encoder.getVelocity();
    }

    public void manualDrive(int direction) {
        double angle = getAngleDeg();

        double speed;
        if (direction == 0) {
            speed = 0.0;
        } else {
            speed = direction * getSpeedForAngle(angle, direction);
        }

        if (angle <= MIN_ANGLE_DEG && speed < 0.0) {
            speed = 0.0;   
        }
        if (angle >= MAX_ANGLE_DEG && speed > 0.0) {
            speed = 0.0;   
        }

        double armAngleRad = Math.toRadians(angle);
        double gravityFF   = kG * Math.sin(armAngleRad);

        double output = speed + gravityFF;

        output = clamp(output, -0.40, 0.40);

        motor.set(output);
    }

    public void stop() {
        motor.set(0.0);
    }

    public boolean isHomed() {
        return isHomed;
    }

    public boolean isAtHomeSwitch() {
        if (!USE_LIMIT_SWITCH || limitSwitch == null) return false;
        return !limitSwitch.get(); 
    }

    private void resetEncoder() {
        encoder.setPosition(0.0);
        isHomed = true;
    }

    public Command homeCommand() {
        if (USE_LIMIT_SWITCH) {
            return Commands.runOnce(() -> {
                    }, this)
                    .andThen(
                        Commands.run(() -> motor.set(HOMING_SPEED), this)
                               .until(this::isAtHomeSwitch)
                    )
                    .finallyDo((interrupted) -> {
                        motor.set(0.0);
                        if (!interrupted) resetEncoder();
                    })
                    .withName("Arm Home (switch)");
        } else {
            return Commands.runOnce(() -> {
                        stallTimer.reset();
                        stallTimer.start();
                    }, this)
                    .andThen(
                        Commands.run(() -> motor.set(HOMING_SPEED), this)
                               .until(() -> {
                                   if (stallTimer.get() < HOMING_IGNORE_TIME) return false;

                                   boolean stalled = Math.abs(getVelocityDegPerSec()) < STALL_VELOCITY_THRESH;
                                   return stalled && stallTimer.get() > (HOMING_IGNORE_TIME + STALL_TIME_THRESH);
                               })
                    )
                    .finallyDo((interrupted) -> {
                        motor.set(0.0);
                        stallTimer.stop();
                        if (!interrupted) resetEncoder();
                    })
                    .withTimeout(3.0)  
                    .withName("Arm Home (stall)");
        }
    }

    @Override
    public void periodic() {
        if (USE_LIMIT_SWITCH && isAtHomeSwitch()) {
            resetEncoder();
        }

        SmartDashboard.putNumber("Arm/AngleDeg",      getAngleDeg());
        SmartDashboard.putNumber("Arm/VelocityDegS",  getVelocityDegPerSec());
        SmartDashboard.putNumber("Arm/MotorOutput",    motor.getAppliedOutput());
        SmartDashboard.putNumber("Arm/CurrentAmps",    motor.getOutputCurrent());
        SmartDashboard.putBoolean("Arm/IsHomed",       isHomed);
        SmartDashboard.putBoolean("Arm/AtHomeSwitch",  isAtHomeSwitch());
    }

    private double getSpeedForAngle(double angle, int direction) {
        double distToLimit;
        if (direction > 0) {
            distToLimit = MAX_ANGLE_DEG - angle;
        } else {
            distToLimit = angle - MIN_ANGLE_DEG;
        }

        if (distToLimit < SOFT_LIMIT_ZONE_DEG) {
            return MANUAL_SPEED_SLOW;  
        }
        return MANUAL_SPEED;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}