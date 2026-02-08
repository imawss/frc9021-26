package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterProfile {
    // Metadata
    private final String name;
    private final String description;
    
    // Mechanical configuration
    private final double angleDegrees;
    private final double launchHeightMeters;
    private final double targetHeightMeters;
    
    // Ballistics mapping
    private final InterpolatingDoubleTreeMap distanceToRPM;
    
    // Operating constraints
    private final double minSafeDistance;
    private final double maxSafeDistance;
    private final double defaultRPM;
    
    /**
     * Create a new shooter profile.
     * 
     * @param name Short identifier (e.g., "BALANCED", "STEEP_CLOSE")
     * @param description Human-readable explanation (e.g., "45° All-Purpose (1.5-5.0m)")
     * @param angleDegrees Mechanical shooter angle from horizontal
     * @param launchHeightMeters Height of shooter wheel center above floor
     * @param targetHeightMeters Height of goal center above floor
     * @param distanceToRPM Map of horizontal distance (m) to wheel RPM
     * @param minSafeDistance Minimum reliable shooting distance (m)
     * @param maxSafeDistance Maximum reliable shooting distance (m)
     * @param defaultRPM Fallback RPM if distance is out of range
     */
    public ShooterProfile(
        String name,
        String description,
        double angleDegrees,
        double launchHeightMeters,
        double targetHeightMeters,
        InterpolatingDoubleTreeMap distanceToRPM,
        double minSafeDistance,
        double maxSafeDistance,
        double defaultRPM
    ) {
        this.name = name;
        this.description = description;
        this.angleDegrees = angleDegrees;
        this.launchHeightMeters = launchHeightMeters;
        this.targetHeightMeters = targetHeightMeters;
        this.distanceToRPM = distanceToRPM;
        this.minSafeDistance = minSafeDistance;
        this.maxSafeDistance = maxSafeDistance;
        this.defaultRPM = defaultRPM;
    }
    
    // ========================================
    // GETTERS
    // ========================================
    
    public String getName() {
        return name;
    }
    
    public String getDescription() {
        return description;
    }
    
    public double getAngleDegrees() {
        return angleDegrees;
    }
    
    public double getLaunchHeightMeters() {
        return launchHeightMeters;
    }
    
    public double getTargetHeightMeters() {
        return targetHeightMeters;
    }
    
    public double getMinSafeDistance() {
        return minSafeDistance;
    }
    
    public double getMaxSafeDistance() {
        return maxSafeDistance;
    }
    
    public double getDefaultRPM() {
        return defaultRPM;
    }
    
    /**
     * Get wheel RPM for given distance using interpolation.
     * 
     * @param distanceMeters horizontal distance to target
     * @return interpolated wheel RPM (not clamped to safe range)
     */
    public double getRPMForDistance(double distanceMeters) {
        return distanceToRPM.get(distanceMeters);
    }
    
    /**
     * Check if distance is within this profile's safe operating range.
     * 
     * @param distanceMeters horizontal distance to target
     * @return true if distance is between min and max safe distance
     */
    public boolean isDistanceInRange(double distanceMeters) {
        return distanceMeters >= minSafeDistance && distanceMeters <= maxSafeDistance;
    }
    
    /**
     * Get display name for dashboard (includes description).
     * 
     * @return formatted string like "BALANCED - 45° All-Purpose (1.5-5.0m)"
     */
    public String getDisplayName() {
        return name + " - " + description;
    }
    
    @Override
    public String toString() {
        return String.format("ShooterProfile[%s, %.1f°, %.1f-%.1fm]",
            name, angleDegrees, minSafeDistance, maxSafeDistance);
    }
}