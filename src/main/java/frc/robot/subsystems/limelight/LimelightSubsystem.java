package frc.robot.subsystems.limelight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.limelight.LimelightHelpers.RawFiducial;

/**
 * Manages Limelight vision, MegaTag2 odometry fusion, distance measurement,
 * and AprilTag alignment assist.
 */
public class LimelightSubsystem extends SubsystemBase {
    private static final String kLimelightName = "limelight";

    // Proportional gain for rotation-to-target alignment
    private static final double kPAim = 0.035;

    // Vision fusion thresholds — teleop
    private static final int    kMinTagsTeleop     = 1;
    private static final double kMaxDistanceTeleop = 5.0;

    // Vision fusion thresholds — autonomous (stricter)
    private static final int    kMinTagsAuto     = 2;
    private static final double kMaxDistanceAuto = 4.0;

    private CommandSwerveDrivetrain m_drivetrain;
    private boolean m_visionUpdatesEnabled = true;
    private boolean m_isAutoMode           = false;

    public LimelightSubsystem() {
        LimelightHelpers.setLEDMode_ForceOff(kLimelightName);
    }

    // ========== Configuration ==========

    /** Provides the drivetrain reference required for MegaTag2 odometry fusion. */
    public void setDrivetrain(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
    }

    /** Enables or disables vision odometry updates. */
    public void setVisionUpdatesEnabled(boolean enabled) {
        m_visionUpdatesEnabled = enabled;
    }

    /**
     * Switches between auto and teleop vision criteria.
     * Auto mode uses stricter tag count and distance thresholds.
     */
    public void setAutoMode(boolean isAuto) {
        m_isAutoMode = isAuto;
        if (isAuto) {
            setLEDsOn();
        } else {
            setLEDsOff();
        }
    }

    // ========== Limelight Readings ==========

    /** Returns horizontal offset from crosshair to target in degrees. Positive = target is right. */
    public double getHorizontalOffset() {
        return LimelightHelpers.getTX(kLimelightName);
    }

    /** Returns vertical offset from crosshair to target in degrees. */
    public double getVerticalOffset() {
        return LimelightHelpers.getTY(kLimelightName);
    }

    /** Gets the target area (0% to 100% of image). */
    public double getTargetArea() {
        return LimelightHelpers.getTA(kLimelightName);
    }

    /** Returns true if the Limelight has a valid target lock. */
    public boolean hasValidTarget() {
        return LimelightHelpers.getTV(kLimelightName);
    }

    // ========== Distance Calculation ==========

    /**
    * Returns distance from the robot to the primary AprilTag in meters.
    * Uses the Limelight's 3D fiducial solve which is more accurate than
    * the trigonometric estimate. Returns -1.0 if no target is visible.
    */
    public double getDistanceToTarget() {
        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(kLimelightName);

        if (fiducials.length == 0) {
            return -1.0;
        }

        return fiducials[0].distToRobot;
    }

    /**
     * Returns distance to target with bounds checking.
     * Falls back to 3.0 m if the measurement is invalid.
     */
    public double getDistanceToTargetSafe() {
        double distance = getDistanceToTarget();

        if (distance < 0 || distance > 10.0 || Double.isNaN(distance) || Double.isInfinite(distance)) {
            return 3.0;
        }

        return distance;
    }

    // ========== Alignment ==========

    /**
     * Calculates a rotational velocity for aligning to the current target.
     * Returns 0 if no target is visible.
     */
    public double calculateAimVelocity(double maxAngularRate) {
        if (!hasValidTarget()) {
            return 0.0;
        }

        return -getHorizontalOffset() * kPAim * maxAngularRate;
    }

    // ========== MegaTag2 Vision Fusion ==========

    /** Returns the latest MegaTag2 pose estimate in the WPILib Blue alliance frame. */
    public LimelightHelpers.PoseEstimate getMegaTag2Estimate() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kLimelightName);
    }

    /** Validates and applies a MegaTag2 pose estimate to the drivetrain Kalman filter. */
    private void processVisionMeasurements() {
        if (m_drivetrain == null || !m_visionUpdatesEnabled) {
            return;
        }

        LimelightHelpers.PoseEstimate mt2 = getMegaTag2Estimate();

        if (mt2 != null && isValidVisionMeasurement(mt2)) {
            double distance = getDistanceToTargetSafe();
            m_drivetrain.addVisionMeasurementWithDistance(
                mt2.pose,
                mt2.timestampSeconds,
                distance,
                mt2.tagCount
            );
            SmartDashboard.putBoolean("Vision/Active", true);
        } else {
            SmartDashboard.putBoolean("Vision/Active", false);
        }
    }

    /** Returns true if the pose estimate meets the current mode's quality thresholds. */
    private boolean isValidVisionMeasurement(LimelightHelpers.PoseEstimate estimate) {
        int    minTags     = m_isAutoMode ? kMinTagsAuto     : kMinTagsTeleop;
        double maxDistance = m_isAutoMode ? kMaxDistanceAuto : kMaxDistanceTeleop;

        if (estimate.tagCount < minTags) {
            return false;
        }

        double distance = getDistanceToTargetSafe();
        if (distance > maxDistance || estimate.avgTagDist > maxDistance) {
            return false;
        }

        // Reject origin pose which indicates uninitialized data
        if (estimate.pose.getX() == 0 && estimate.pose.getY() == 0) {
            return false;
        }

        return true;
    }

    // ========== LED Control ==========

    public void setLEDsOn() {
        LimelightHelpers.setLEDMode_ForceOn(kLimelightName);
    }

    public void setLEDsOff() {
        LimelightHelpers.setLEDMode_ForceOff(kLimelightName);
    }

    public void setPipeline(int pipeline) {
        LimelightHelpers.setPipelineIndex(kLimelightName, pipeline);
    }

    // ========== Periodic ==========

    @Override
    public void periodic() {
        try {
            // MegaTag2 requires robot yaw to be sent every loop before requesting the estimate
            if (m_drivetrain != null) {
                double yawDegrees = m_drivetrain.getState().Pose.getRotation().getDegrees();
                LimelightHelpers.SetRobotOrientation_NoFlush(
                    kLimelightName,
                    yawDegrees, 0,
                    0, 0,
                    0, 0
                );
            }

            processVisionMeasurements();

            // Essential driver-facing telemetry
            SmartDashboard.putBoolean("Vision/HasTarget", hasValidTarget());
            SmartDashboard.putNumber("Vision/HorizontalOffset", getHorizontalOffset());
            SmartDashboard.putNumber("Vision/Distance", getDistanceToTarget());

            LimelightHelpers.PoseEstimate mt2 = getMegaTag2Estimate();
            if (mt2 != null) {
                SmartDashboard.putNumber("Vision/TagCount", mt2.tagCount);
            }
        } catch (Exception e) {
            // Prevent a Limelight error from crashing the robot
        }
    }
}