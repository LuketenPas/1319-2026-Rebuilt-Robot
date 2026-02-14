package frc.robot.subsystems.limelight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class LimelightSubsystem extends SubsystemBase {
    private static final String LIMELIGHT_NAME = "limelight";
    
    // Proportional control constant
    private static final double kP_AIM = 0.035;
    
    // Distance Calculation Constants
    private static final double LIMELIGHT_HEIGHT_METERS = 0.5;
    private static final double TARGET_HEIGHT_METERS = 1.5;
    private static final double LIMELIGHT_MOUNT_ANGLE_DEGREES = 30.0;
    
    // PRO FEATURE: Vision measurement configuration
    private static final int MIN_TAGS_FOR_VISION_UPDATE = 1;  // Lowered to 1 for auto (was 2)
    private static final double MAX_DISTANCE_FOR_VISION_UPDATE = 5.0;  // Max distance in meters
    
    // PRO FEATURE: Separate standards for auto vs teleop
    private static final int MIN_TAGS_FOR_AUTO = 2;  // Higher confidence needed in auto
    private static final double MAX_DISTANCE_FOR_AUTO = 4.0;  // Closer range in auto
    
    private CommandSwerveDrivetrain m_drivetrain = null;
    private boolean m_visionUpdatesEnabled = true;  // Can disable if needed
    private boolean m_isAutoMode = false;  // Track if we're in autonomous
    
    public LimelightSubsystem() {
        LimelightHelpers.setLEDMode_ForceOff(LIMELIGHT_NAME);
    }
    
    /**
     * PRO FEATURE: Set the drivetrain reference for vision measurement integration
     */
    public void setDrivetrain(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
    }
    
    /**
     * Enable or disable vision odometry updates
     * Useful for debugging or if vision is unreliable
     */
    public void setVisionUpdatesEnabled(boolean enabled) {
        m_visionUpdatesEnabled = enabled;
        System.out.println("Vision updates " + (enabled ? "ENABLED" : "DISABLED"));
    }
    
    /**
     * Tell the subsystem we're in autonomous mode
     * This uses stricter vision measurement criteria
     */
    public void setAutoMode(boolean isAuto) {
        m_isAutoMode = isAuto;
        if (isAuto) {
            // Turn LEDs on during auto for better AprilTag detection
            setLEDsOn();
            System.out.println("Limelight: AUTO MODE - Using strict vision criteria");
        } else {
            setLEDsOff();
            System.out.println("Limelight: TELEOP MODE - Using standard vision criteria");
        }
    }
    
    public double getHorizontalOffset() {
        return LimelightHelpers.getTX(LIMELIGHT_NAME);
    }
    
    public double getVerticalOffset() {
        return LimelightHelpers.getTY(LIMELIGHT_NAME);
    }
    
    public double getTargetArea() {
        return LimelightHelpers.getTA(LIMELIGHT_NAME);
    }
    
    public boolean hasValidTarget() {
        return LimelightHelpers.getTV(LIMELIGHT_NAME);
    }
    
    public double calculateAimVelocity(double maxAngularRate) {
        if (!hasValidTarget()) {
            return 0.0;
        }
        
        double targetingAngularVelocity = getHorizontalOffset() * kP_AIM;
        targetingAngularVelocity *= maxAngularRate;
        targetingAngularVelocity *= -1.0;
        return targetingAngularVelocity;
    }
    
    public double getDistanceToTarget() {
        if (!hasValidTarget()) {
            return -1.0;
        }
        
        double ty = getVerticalOffset();
        double angleToTargetDegrees = LIMELIGHT_MOUNT_ANGLE_DEGREES + ty;
        double angleToTargetRadians = Math.toRadians(angleToTargetDegrees);
        double heightDifference = TARGET_HEIGHT_METERS - LIMELIGHT_HEIGHT_METERS;
        double distance = heightDifference / Math.tan(angleToTargetRadians);
        
        return distance;
    }
    
    public double getDistanceToTargetSafe() {
        double distance = getDistanceToTarget();
        
        if (distance < 0 || distance > 10.0 || Double.isNaN(distance) || Double.isInfinite(distance)) {
            return 3.0;
        }
        
        return distance;
    }
    
    /**
     * PRO FEATURE: Get MegaTag2 pose estimate for vision measurement integration
     */
    public LimelightHelpers.PoseEstimate getMegaTag2Estimate() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_NAME);
    }
    
    /**
     * PRO FEATURE: Process vision measurements and integrate with drivetrain odometry
     * Now public so it can be called on-demand or monitored
     */
    public void processVisionMeasurements() {
        if (m_drivetrain == null || !m_visionUpdatesEnabled) {
            SmartDashboard.putBoolean("Limelight/VisionUpdateApplied", false);
            return;
        }
        
        // Get MegaTag2 pose estimate
        LimelightHelpers.PoseEstimate mt2 = getMegaTag2Estimate();
        
        // Validate the measurement before using it
        if (mt2 != null && isValidVisionMeasurement(mt2)) {
            double distance = getDistanceToTargetSafe();
            
            // Add vision measurement to drivetrain with dynamic standard deviations
            m_drivetrain.addVisionMeasurementWithDistance(
                mt2.pose,
                mt2.timestampSeconds,
                distance,
                mt2.tagCount
            );
            
            SmartDashboard.putBoolean("Limelight/VisionUpdateApplied", true);
            SmartDashboard.putNumber("Limelight/LastUpdateTagCount", mt2.tagCount);
            SmartDashboard.putNumber("Limelight/LastUpdateDistance", distance);
        } else {
            SmartDashboard.putBoolean("Limelight/VisionUpdateApplied", false);
        }
    }
    
    /**
     * PRO FEATURE: Validates vision measurement before applying to odometry
     * Uses different criteria for auto vs teleop
     */
    private boolean isValidVisionMeasurement(LimelightHelpers.PoseEstimate estimate) {
        // Use stricter criteria in auto mode
        int minTags = m_isAutoMode ? MIN_TAGS_FOR_AUTO : MIN_TAGS_FOR_VISION_UPDATE;
        double maxDistance = m_isAutoMode ? MAX_DISTANCE_FOR_AUTO : MAX_DISTANCE_FOR_VISION_UPDATE;
        
        // Must have minimum number of tags
        if (estimate.tagCount < minTags) {
            SmartDashboard.putString("Limelight/RejectReason", "Not enough tags (" + estimate.tagCount + " < " + minTags + ")");
            return false;
        }
        
        // Must be within reasonable distance
        double distance = getDistanceToTargetSafe();
        if (distance > maxDistance) {
            SmartDashboard.putString("Limelight/RejectReason", "Too far (" + String.format("%.2f", distance) + "m > " + maxDistance + "m)");
            return false;
        }
        
        // Pose must not be at origin (indicates invalid data)
        if (estimate.pose.getX() == 0 && estimate.pose.getY() == 0) {
            SmartDashboard.putString("Limelight/RejectReason", "Pose at origin (invalid)");
            return false;
        }
        
        // Average tag distance should be reasonable
        if (estimate.avgTagDist > maxDistance) {
            SmartDashboard.putString("Limelight/RejectReason", "Avg tag distance too far");
            return false;
        }
        
        SmartDashboard.putString("Limelight/RejectReason", "ACCEPTED");
        return true;
    }
    
    public void setLEDsOn() {
        LimelightHelpers.setLEDMode_ForceOn(LIMELIGHT_NAME);
    }
    
    public void setLEDsOff() {
        LimelightHelpers.setLEDMode_ForceOff(LIMELIGHT_NAME);
    }
    
    public void setPipeline(int pipeline) {
        LimelightHelpers.setPipelineIndex(LIMELIGHT_NAME, pipeline);
    }
    
    @Override
    public void periodic() {
        try {
            boolean hasTarget = hasValidTarget();
            double tx = getHorizontalOffset();
            double ty = getVerticalOffset();
            double ta = getTargetArea();
            double distance = getDistanceToTarget();
            double distanceSafe = getDistanceToTargetSafe();
            
            // Log telemetry to SmartDashboard
            SmartDashboard.putBoolean("Limelight/HasTarget", hasTarget);
            SmartDashboard.putNumber("Limelight/TX", tx);
            SmartDashboard.putNumber("Limelight/TY", ty);
            SmartDashboard.putNumber("Limelight/TA", ta);
            SmartDashboard.putNumber("Limelight/Distance_Meters", distance);
            SmartDashboard.putNumber("Limelight/Distance_Safe", distanceSafe);
            SmartDashboard.putBoolean("Limelight/VisionEnabled", m_visionUpdatesEnabled);
            SmartDashboard.putBoolean("Limelight/AutoMode", m_isAutoMode);
            
            // PRO FEATURE: Process vision measurements for odometry fusion
            processVisionMeasurements();
            
            // Log MegaTag2 info if available
            LimelightHelpers.PoseEstimate mt2 = getMegaTag2Estimate();
            if (mt2 != null) {
                SmartDashboard.putNumber("Limelight/MegaTag2/TagCount", mt2.tagCount);
                SmartDashboard.putNumber("Limelight/MegaTag2/AvgTagDist", mt2.avgTagDist);
                SmartDashboard.putNumber("Limelight/MegaTag2/X", mt2.pose.getX());
                SmartDashboard.putNumber("Limelight/MegaTag2/Y", mt2.pose.getY());
                SmartDashboard.putNumber("Limelight/MegaTag2/Rotation", mt2.pose.getRotation().getDegrees());
            }
            
        } catch (Exception e) {
            // Silently catch any errors to prevent crash
            System.err.println("Limelight periodic error: " + e.getMessage());
        }
    }
}