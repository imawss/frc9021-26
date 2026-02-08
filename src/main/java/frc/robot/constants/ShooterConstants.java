package frc.robot.constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.shooter.ShooterProfile;

import java.util.HashMap;
import java.util.Map;

/**
 * Constants for shooter subsystem.
 * 
 * This file contains:
 * - Hardware IDs and physical properties
 * - Control parameters (PID, feedforward)
 * - Factory methods to create shooter profiles
 * 
 * This does NOT contain the ShooterProfile class itself - that's in its own file.
 */
public final class ShooterConstants {
    private ShooterConstants() {}
    
    public static final int MOTOR_ID = 10;
    public static final String CANBUS = "rio";
    
    // Mechanism properties
    public static final double GEAR_RATIO = 0; 
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4.0);

    public static final double kP = 0.1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.0;   // Static friction (volts)
    public static final double kV = 0.12;  // Velocity feedforward (volts per rot/sec)
    public static final double kA = 0.01;  // Acceleration feedforward (volts per rot/sec²)
    
    public static final double VELOCITY_TOLERANCE_RPM = 50.0;

    public static final double VELOCITY_UPDATE_FREQ = 100.0;
    public static final double VOLTAGE_UPDATE_FREQ = 4.0;

    public static final double LAUNCH_HEIGHT_METERS = 0.58;  // Floor to wheel center
    public static final double TARGET_HEIGHT_METERS = 2.05;  // Floor to goal center

    public static final String DEFAULT_PROFILE_NAME = "BALANCED";
    
    public static Map<String, ShooterProfile> createAllProfiles() {
        Map<String, ShooterProfile> profiles = new HashMap<>();
        
        profiles.put("BALANCED", createBalancedProfile());
        profiles.put("STEEP_CLOSE", createSteepCloseProfile());
        profiles.put("FLAT_LONG", createFlatLongProfile());
        profiles.put("EXPERIMENTAL", createExperimentalProfile());
        
        return profiles;
    }
    
    private static ShooterProfile createBalancedProfile() {
        InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();
        
        // Distance (m) → Wheel RPM
        // REPLACE WITH EMPIRICAL DATA
        map.put(1.5, 2600.0);
        map.put(2.0, 3000.0);
        map.put(2.5, 3450.0);
        map.put(3.0, 3950.0);
        map.put(3.5, 4500.0);
        map.put(4.0, 5100.0);
        map.put(4.5, 5750.0);
        map.put(5.0, 6450.0);
        
        return new ShooterProfile(
            "BALANCED",
            "45° All-Purpose (1.5-5.0m)",
            45.0,  // angle
            LAUNCH_HEIGHT_METERS,
            TARGET_HEIGHT_METERS,
            map,
            1.5,   // min distance
            5.0,   // max distance
            3800.0 // default RPM
        );
    }

    private static ShooterProfile createSteepCloseProfile() {
        InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();
        
        map.put(1.0, 2200.0);
        map.put(1.5, 2500.0);
        map.put(2.0, 2900.0);
        map.put(2.5, 3400.0);
        map.put(3.0, 4000.0);
        map.put(3.5, 4700.0);
        
        return new ShooterProfile(
            "STEEP_CLOSE",
            "60° Over Defense (1.0-3.5m)",
            60.0,
            LAUNCH_HEIGHT_METERS,
            TARGET_HEIGHT_METERS,
            map,
            1.0,
            3.5,
            3000.0
        );
    }
    
    private static ShooterProfile createFlatLongProfile() {
        InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();
        
        map.put(2.5, 3800.0);
        map.put(3.0, 4200.0);
        map.put(3.5, 4650.0);
        map.put(4.0, 5150.0);
        map.put(4.5, 5700.0);
        map.put(5.0, 6300.0);
        map.put(5.5, 6950.0);
        map.put(6.0, 7650.0);
        
        return new ShooterProfile(
            "FLAT_LONG",
            "35° Long Range (2.5-6.0m)",
            35.0,
            LAUNCH_HEIGHT_METERS,
            TARGET_HEIGHT_METERS,
            map,
            2.5,
            6.0,
            4500.0
        );
    }
    
    private static ShooterProfile createExperimentalProfile() {
        InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();
        
        // Start with balanced data, then modify
        map.put(1.5, 2700.0);
        map.put(2.0, 3100.0);
        map.put(2.5, 3550.0);
        map.put(3.0, 4050.0);
        map.put(3.5, 4600.0);
        map.put(4.0, 5200.0);
        map.put(4.5, 5850.0);
        
        return new ShooterProfile(
            "EXPERIMENTAL",
            "Test Config (47° - USE CAUTION)",
            47.0,
            LAUNCH_HEIGHT_METERS,
            TARGET_HEIGHT_METERS,
            map,
            1.5,
            4.5,
            3900.0
        );
    }
}