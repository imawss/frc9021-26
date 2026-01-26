package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import frc.robot.constants.IntakeConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Pivot Intake Subsystem
 * Robot hep intake kapalı başlatılmalı! Eğer öyle başlatılmadıysa elle kapatılıp calibrate tuşuna basılmalı.
 */
public class IntakeSubsystem extends SubsystemBase {

    private final SparkMax intakeMotor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController pidController;

    private double targetPosition = IntakeConstants.INTAKE_RETRACTED_DEGREES;
    
    /** Manuel kontrol modu aktif mi? */
    private boolean manualMode = false;
    
    public IntakeSubsystem() {
        intakeMotor = new SparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake)
              .smartCurrentLimit(30)
              .inverted(false);
        
        double positionConversionFactor = 360.0 / IntakeConstants.GEAR_RATIO;
        config.encoder
              .positionConversionFactor(positionConversionFactor)
              .velocityConversionFactor(positionConversionFactor / 60.0);
        
        config.closedLoop
              .pid(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD)
              .positionWrappingEnabled(false);
        
        config.softLimit
              .forwardSoftLimit(IntakeConstants.INTAKE_EXTENDED_DEGREES + 10)
              .forwardSoftLimitEnabled(true)
              .reverseSoftLimit(IntakeConstants.INTAKE_RETRACTED_DEGREES - 10)
              .reverseSoftLimitEnabled(true);

        intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = intakeMotor.getEncoder();
        pidController = intakeMotor.getClosedLoopController();

        // İlk pozisyon (Intake fiziksel olarak retracted pozisyonda olmalı!)
        encoder.setPosition(IntakeConstants.INTAKE_RETRACTED_DEGREES);
    }
    
    public void extend() {
        setTargetPosition(IntakeConstants.INTAKE_EXTENDED_DEGREES);
    }
    
    public void retract() {
        setTargetPosition(IntakeConstants.INTAKE_RETRACTED_DEGREES);
    }
    
    public void toggle() {
        if (isRetracted()) {
            extend();
        } else {
            retract();
        }
    }

    public void setSpeed(double speed) {
        manualMode = true;
        intakeMotor.set(speed);
        targetPosition = encoder.getPosition(); // Mevcut pozisyonu hedef yap
    }
    
    public void stop() {
        intakeMotor.stopMotor();
        targetPosition = encoder.getPosition();
    }
    
    /**
     * ÖNEMLİ: Sadece intake fiziksel olarak retracted pozisyonda iken kullanın!
     */
    public void calibrate() {
        encoder.setPosition(IntakeConstants.INTAKE_RETRACTED_DEGREES);
        targetPosition = IntakeConstants.INTAKE_RETRACTED_DEGREES;
        manualMode = false;
        System.out.println("Intake encoder calibrated to " + IntakeConstants.INTAKE_RETRACTED_DEGREES + " degrees");
    }
    
    public boolean isExtended() {
        return Math.abs(encoder.getPosition() - IntakeConstants.INTAKE_EXTENDED_DEGREES) 
               < IntakeConstants.POSITION_TOLERANCE;
    }
    
    public boolean isRetracted() {
        return Math.abs(encoder.getPosition() - IntakeConstants.INTAKE_RETRACTED_DEGREES) 
               < IntakeConstants.POSITION_TOLERANCE;
    }
    
    public boolean atTarget() {
        return Math.abs(encoder.getPosition() - targetPosition) < IntakeConstants.POSITION_TOLERANCE;
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    public double getVelocity() {
        return encoder.getVelocity();
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    @Deprecated
    public void resetEncoder() {
        calibrate();
    }
    
    public double getMotorCurrent() {
        return intakeMotor.getOutputCurrent();
    }
    
    public boolean isManualMode() {
        return manualMode;
    }

    private void setTargetPosition(double positionDeg) {
        if (Math.abs(targetPosition - positionDeg) < 0.1) {
            return;
        }
        
        manualMode = false;
        targetPosition = positionDeg;
        
        pidController.setSetpoint(targetPosition, ControlType.kPosition, 
                                   ClosedLoopSlot.kSlot0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/Position (deg)", getPosition());
        SmartDashboard.putNumber("Intake/Target Position (deg)", targetPosition);
        SmartDashboard.putNumber("Intake/Velocity (deg/s)", getVelocity());
        SmartDashboard.putBoolean("Intake/Is Extended", isExtended());
        SmartDashboard.putBoolean("Intake/Is Retracted", isRetracted());
        SmartDashboard.putBoolean("Intake/At Target", atTarget());
        SmartDashboard.putNumber("Intake/Motor Current", getMotorCurrent());
        SmartDashboard.putBoolean("Intake/Manual Mode", manualMode);
        
        boolean highCurrent = getMotorCurrent() > 25.0;
        SmartDashboard.putBoolean("Intake/High Current Warning", highCurrent);
        
        boolean isStalled = !manualMode && 
                           Math.abs(getVelocity()) < 0.5 && 
                           !atTarget() &&
                           Math.abs(targetPosition - getPosition()) > IntakeConstants.POSITION_TOLERANCE;
        
        SmartDashboard.putBoolean("Intake/Stalled", isStalled);

        boolean outOfBounds = getPosition() < (IntakeConstants.INTAKE_RETRACTED_DEGREES - 15) ||
                             getPosition() > (IntakeConstants.INTAKE_EXTENDED_DEGREES + 15);
        SmartDashboard.putBoolean("Intake/Out of Bounds", outOfBounds);
    }
}