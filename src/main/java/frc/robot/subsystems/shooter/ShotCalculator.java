package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * Calculates optimal hood angle based on distance to target.
 * Uses interpolation between tested data points for smooth transitions.
 * Shooter speed remains constant.
 */
public class ShotCalculator {
    
    // Interpolating map for smooth transitions between tested distances
    private final InterpolatingDoubleTreeMap hoodAngleMap;
    
    // Fallback constant if distance is invalid
    private static final double DEFAULT_HOOD_ANGLE = 0.15;  // rotations
    
    // Valid shooting range limits
    private static final double MIN_SHOOTING_DISTANCE = 0.5;  // meters
    private static final double MAX_SHOOTING_DISTANCE = 6.0;  // meters
    
    public ShotCalculator() {
        hoodAngleMap = new InterpolatingDoubleTreeMap();
        // Format: distance in meters -> hood angle in rotations
        populateHoodAngleTable();
    }
    
    /**
     * Populates the hood angle lookup table with tested data.
     * 
     * IMPORTANT: These are PLACEHOLDER values! You MUST test your robot
     * and replace these with real data from practice.
     * 
     * Testing procedure:
     * 1. Place robot at known distance from target (use measuring tape)
     * 2. Set shooter to your standard speed (e.g., 0.7)
     * 3. Manually adjust hood until shots are consistent
     * 4. Record: distance and hood angle (rotations)
     * 5. Repeat for distances: 1m, 1.5m, 2m, 2.5m, 3m, 3.5m, 4m, 5m
     * 6. Enter those values into this table
     */
    private void populateHoodAngleTable() {
        // HOOD ANGLE TABLE
        // Distance (meters) -> Hood Angle (rotations)
        // Lower angle = flatter shot, higher angle = more arc
        
        hoodAngleMap.put(1.0, 0.05);   // Close shot - low angle
        hoodAngleMap.put(1.5, 0.08);   
        hoodAngleMap.put(2.0, 0.10);   // Medium-close
        hoodAngleMap.put(2.5, 0.12);
        hoodAngleMap.put(3.0, 0.15);   // Medium range
        hoodAngleMap.put(3.5, 0.17);
        hoodAngleMap.put(4.0, 0.20);   // Far shot
        hoodAngleMap.put(5.0, 0.23);   // Very far - high angle
        hoodAngleMap.put(6.0, 0.25);   // Max distance
    }
    
    /**
     * Calculates the optimal hood angle for the given distance.
     * Uses interpolation for smooth transitions between data points.
     * 
     * @param distanceMeters Distance to target in meters
     * @return Hood angle in rotations (clamped to safe limits)
     */
    public double calculateHoodAngle(double distanceMeters) {
        // Validate distance
        if (!isValidDistance(distanceMeters)) {
            System.out.println("Invalid distance for shot calculation: " + distanceMeters + "m, using default");
            return DEFAULT_HOOD_ANGLE;
        }
        
        // Get interpolated value
        double angle = hoodAngleMap.get(distanceMeters);
        
        // Clamp to physical limits (from HoodSubsystem)
        return Math.max(0.0, Math.min(0.25, angle));
    }
    
    /**
     * Checks if the given distance is within valid shooting range.
     * 
     * @param distanceMeters Distance to target in meters
     * @return true if distance is valid for shooting
     */
    public boolean isValidDistance(double distanceMeters) {
        return distanceMeters >= MIN_SHOOTING_DISTANCE 
            && distanceMeters <= MAX_SHOOTING_DISTANCE
            && !Double.isNaN(distanceMeters)
            && !Double.isInfinite(distanceMeters);
    }
    
    /**
     * Checks if we're within optimal shooting range (closer is better).
     * 
     * @param distanceMeters Distance to target in meters
     * @return true if within optimal range
     */
    public boolean isOptimalRange(double distanceMeters) {
        return distanceMeters >= 1.5 && distanceMeters <= 4.0;
    }
    
    /**
     * Gets the minimum shooting distance.
     */
    public double getMinShootingDistance() {
        return MIN_SHOOTING_DISTANCE;
    }
    
    /**
     * Gets the maximum shooting distance.
     */
    public double getMaxShootingDistance() {
        return MAX_SHOOTING_DISTANCE;
    }
}