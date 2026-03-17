package frc.robot.constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.shooter.ShooterProfile;

import java.util.HashMap;
import java.util.Map;

public final class ShooterConstants {
    private ShooterConstants() {}

    public static final int MOTOR_ID = 25;

    public static final double GEAR_RATIO = 1.0;

    public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(2.0);
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4.0);

    public static final double kP_TALON = 0.05;
    public static final double kI_TALON = 0.0;
    public static final double kD_TALON = 0.0;
    public static final double kV_TALON = 0.12;
    public static final double kS_TALON = 0.10;

    public static final double SPINUP_WAIT_SECONDS = 0.75;

    public static final double kFF = 0.0002;

    public static final double VELOCITY_TOLERANCE_RPM = 100.0;
    
    public static final double MAX_OUTPUT = 1.0;

    public static final double LAUNCH_HEIGHT_METERS = 0.58;
    public static final double TARGET_HEIGHT_METERS = 2.05;

    public static final String DEFAULT_PROFILE_NAME = "STANDART";

    public static Map<String, ShooterProfile> createAllProfiles() {
        Map<String, ShooterProfile> profiles = new HashMap<>();
        profiles.put("STANDART",     createStandartProfile());
        profiles.put("EXPERIMENTAL", createExperimentalProfile());
        return profiles;
    }

    private static ShooterProfile createStandartProfile() {
        InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();
        map.put(1.5, 2600.0);
        map.put(2.0, 3000.0);
        map.put(2.5, 3450.0);
        map.put(3.0, 3950.0);
        map.put(3.5, 4500.0);
        map.put(4.0, 5100.0);
        map.put(4.5, 5750.0);
        map.put(5.0, 6450.0);
        return new ShooterProfile("STANDART", "Standart mil düzeni",
            45.0, LAUNCH_HEIGHT_METERS, TARGET_HEIGHT_METERS, map, 1.5, 5.0, 3800.0);
    }

    private static ShooterProfile createExperimentalProfile() {
        InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();
        map.put(1.5, 2700.0);
        map.put(2.0, 3100.0);
        map.put(2.5, 3550.0);
        map.put(3.0, 4050.0);
        map.put(3.5, 4600.0);
        map.put(4.0, 5200.0);
        map.put(4.5, 5850.0);
        return new ShooterProfile("EXPERIMENTAL", "Test Config (47° - USE CAUTION)",
            47.0, LAUNCH_HEIGHT_METERS, TARGET_HEIGHT_METERS, map, 1.5, 4.5, 3900.0);
    }
}