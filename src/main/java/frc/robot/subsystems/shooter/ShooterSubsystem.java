package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

import java.util.Map;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX motor;
    private final VelocityVoltage velocityControl;
    private final StatusSignal<AngularVelocity> velocitySignal;

    private final Map<String, ShooterProfile> availableProfiles;
    private final SendableChooser<String> profileChooser;
    private ShooterProfile activeProfile;
    private String lastSelectedProfileName = "";

    private double targetRPM = 0.0;
    private boolean motorConfigured = false;
    
    public ShooterSubsystem() {
        motor = new TalonFX(ShooterConstants.MOTOR_ID, ShooterConstants.CANBUS);
        
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = ShooterConstants.kP;
        config.Slot0.kI = ShooterConstants.kI;
        config.Slot0.kD = ShooterConstants.kD;
        config.Slot0.kS = ShooterConstants.kS;
        config.Slot0.kV = ShooterConstants.kV;
        config.Slot0.kA = ShooterConstants.kA;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;
        
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++) {
            status = motor.getConfigurator().apply(config);
            if (status.isOK()) {
                motorConfigured = true;
                break;
            }
            Timer.delay(0.01);
        }
        
        if (!motorConfigured) {
            DriverStation.reportError(
                "Shooter motor (ID " + ShooterConstants.MOTOR_ID + ") config failed: " + status,
                false
            );
        }
        
        velocityControl = new VelocityVoltage(0).withSlot(0);
        velocitySignal = motor.getVelocity();
        velocitySignal.setUpdateFrequency(ShooterConstants.VELOCITY_UPDATE_FREQ);
        motor.optimizeBusUtilization();

        availableProfiles = ShooterConstants.createAllProfiles();
        profileChooser = new SendableChooser<>();
        
        // Populate dashboard chooser
        boolean defaultSet = false;
        for (Map.Entry<String, ShooterProfile> entry : availableProfiles.entrySet()) {
            String profileName = entry.getKey();
            ShooterProfile profile = entry.getValue();
            
            if (profileName.equals(ShooterConstants.DEFAULT_PROFILE_NAME) && !defaultSet) {
                profileChooser.setDefaultOption(profile.getDisplayName(), profileName);
                defaultSet = true;
            } else {
                profileChooser.addOption(profile.getDisplayName(), profileName);
            }
        }
        
        SmartDashboard.putData("Shooter/Profile Selector", profileChooser);
        
        // Load default profile
        setActiveProfile(ShooterConstants.DEFAULT_PROFILE_NAME);
        
        DriverStation.reportWarning(
            "Shooter initialized with " + availableProfiles.size() + " profiles",
            false
        );
    }
    
    @Override
    public void periodic() {
        String selectedProfileName = profileChooser.getSelected();
        if (selectedProfileName != null && !selectedProfileName.equals(lastSelectedProfileName)) {
            setActiveProfile(selectedProfileName);
        }

        if (!motorConfigured) {
            SmartDashboard.putBoolean("Shooter/Motor Configured", false);
        }
        
        // Motor state
        SmartDashboard.putNumber("Shooter/Target RPM", targetRPM);
        SmartDashboard.putNumber("Shooter/Actual RPM", getWheelRPM());
        SmartDashboard.putNumber("Shooter/Error RPM", targetRPM - getWheelRPM());
        SmartDashboard.putBoolean("Shooter/At Target", atTargetVelocity());
        SmartDashboard.putNumber("Shooter/Motor RPM", getMotorRPM());
        
        // Active profile info
        if (activeProfile != null) {
            SmartDashboard.putString("Shooter/Active Profile", activeProfile.getName());
            SmartDashboard.putNumber("Shooter/Profile Angle (deg)", activeProfile.getAngleDegrees());
            SmartDashboard.putNumber("Shooter/Profile Min Dist (m)", activeProfile.getMinSafeDistance());
            SmartDashboard.putNumber("Shooter/Profile Max Dist (m)", activeProfile.getMaxSafeDistance());
        }
    }
    
    /**
     * Set shooter velocity based on distance to target.
     * Uses active profile's interpolation map.
     * 
     * @param distanceMeters horizontal distance from shooter to target
     */
    public void setVelocityForDistance(double distanceMeters) {
        if (activeProfile == null) {
            DriverStation.reportError("No active shooter profile", false);
            return;
        }
        
        double wheelRPM = getRPMForDistance(distanceMeters);
        setVelocityRPM(wheelRPM);
        
        SmartDashboard.putNumber("Shooter/Last Distance (m)", distanceMeters);
        SmartDashboard.putNumber("Shooter/Last Commanded RPM", wheelRPM);
    }
    
    /**
     * Directly set shooter wheel velocity in RPM.
     * 
     * @param wheelRPM target wheel surface speed in RPM
     */
    public void setVelocityRPM(double wheelRPM) {
        targetRPM = wheelRPM;
        double motorRPM = wheelRPM * ShooterConstants.GEAR_RATIO;
        double motorRPS = motorRPM / 60.0;
        motor.setControl(velocityControl.withVelocity(motorRPS));
    }
    
    /**
     * Stop shooter motor.
     */
    public void stop() {
        targetRPM = 0.0;
        motor.stopMotor();
    }
    
    /**
     * Get current wheel surface speed in RPM.
     */
    public double getWheelRPM() {
        return getMotorRPM() / ShooterConstants.GEAR_RATIO;
    }
    
    /**
     * Get current motor shaft speed in RPM.
     */
    public double getMotorRPM() {
        return velocitySignal.getValueAsDouble() * 60.0;
    }
    
    /**
     * Check if shooter is at target velocity within tolerance.
     */
    public boolean atTargetVelocity() {
        if (targetRPM == 0.0) return false;
        double errorRPM = Math.abs(targetRPM - getWheelRPM());
        return errorRPM < ShooterConstants.VELOCITY_TOLERANCE_RPM;
    }
    
    /**
     * Check if given distance is within active profile's safe range.
     */
    public boolean isDistanceInRange(double distanceMeters) {
        if (activeProfile == null) return false;
        return activeProfile.isDistanceInRange(distanceMeters);
    }
    
    /**
     * Get active profile name.
     */
    public String getActiveProfileName() {
        return activeProfile != null ? activeProfile.getName() : "NONE";
    }
    
    /**
     * Get active profile angle.
     */
    public double getActiveProfileAngle() {
        return activeProfile != null ? activeProfile.getAngleDegrees() : 0.0;
    }
    
    /**
     * Switch to a different shooter profile.
     * Called automatically when user selects from dashboard.
     * 
     * @param profileName name of profile to activate
     */
    public void setActiveProfile(String profileName) {
        if (!availableProfiles.containsKey(profileName)) {
            DriverStation.reportError(
                "Profile '" + profileName + "' not found. Using default.",
                false
            );
            profileName = ShooterConstants.DEFAULT_PROFILE_NAME;
        }
        
        activeProfile = availableProfiles.get(profileName);
        lastSelectedProfileName = profileName;
        
        DriverStation.reportWarning(
            String.format("Shooter profile: %s (%.1f°, %.1f-%.1fm)",
                activeProfile.getName(),
                activeProfile.getAngleDegrees(),
                activeProfile.getMinSafeDistance(),
                activeProfile.getMaxSafeDistance()),
            false
        );
    }
    
    /**
     * Lookup wheel RPM for distance using active profile.
     * Handles out-of-range cases with warnings.
     */
    private double getRPMForDistance(double distance) {
        if (activeProfile == null) {
            DriverStation.reportError("No active profile - using default RPM", false);
            return 3500.0;
        }
        
        // Check bounds
        if (distance < activeProfile.getMinSafeDistance()) {
            DriverStation.reportWarning(
                String.format("Distance %.2fm below min %.2fm - using edge value",
                    distance, activeProfile.getMinSafeDistance()),
                false
            );
            SmartDashboard.putBoolean("Shooter/Distance In Range", false);
            return activeProfile.getRPMForDistance(activeProfile.getMinSafeDistance());
        }
        
        if (distance > activeProfile.getMaxSafeDistance()) {
            DriverStation.reportWarning(
                String.format("Distance %.2fm exceeds max %.2fm - using edge value",
                    distance, activeProfile.getMaxSafeDistance()),
                false
            );
            SmartDashboard.putBoolean("Shooter/Distance In Range", false);
            return activeProfile.getRPMForDistance(activeProfile.getMaxSafeDistance());
        }
        
        SmartDashboard.putBoolean("Shooter/Distance In Range", true);
        
        // Within range - use interpolation
        return activeProfile.getRPMForDistance(distance);
    }
}