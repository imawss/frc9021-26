package frc.robot.constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

public final class ShooterConstants {
    private ShooterConstants() {
    }

    public static final int MOTOR_ID = 25;

    public static final double GEAR_RATIO = 1.0;

    public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(2.0);
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4.0);

    public static final double kP_TALON = 0.05;
    public static final double kI_TALON = 0.0;
    public static final double kD_TALON = 0.0;
    public static final double kV_TALON = 0.12;
    public static final double kS_TALON = 0.10;

    public static final double SPINUP_WAIT_SECONDS = 0.3;

    public static final double kFF = 0.0002;

    public static final double VELOCITY_TOLERANCE_RPM = 300.0;
    public static final double MAX_OUTPUT = 1.0;

    public static final double LAUNCH_HEIGHT_METERS = 0.58;
    public static final double TARGET_HEIGHT_METERS = 2.05;

    public static final double SHOOTER_ANGLE_DEGREES = 45.0;
    public static final double MIN_SAFE_DISTANCE = 1.5;
    public static final double MAX_SAFE_DISTANCE = 5.0;
    public static final double DEFAULT_RPM = 3800.0;

    public static final InterpolatingDoubleTreeMap DISTANCE_TO_RPM_MAP = new InterpolatingDoubleTreeMap();

    static {
        DISTANCE_TO_RPM_MAP.put(1.5, 2700.0);
        DISTANCE_TO_RPM_MAP.put(2.5, 3200.0);
        DISTANCE_TO_RPM_MAP.put(3.0, 3500.0);
        DISTANCE_TO_RPM_MAP.put(4.0, 3650.0);
        DISTANCE_TO_RPM_MAP.put(4.5, 3900.0);
        DISTANCE_TO_RPM_MAP.put(5.0, 4150.0);
    }
}