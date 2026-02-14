package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.shooter.FlyWheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.UptakeSubsystem;

/**
 * Automatically adjusts hood angle based on distance to target.
 * Shooter speed remains constant.
 * Hold button down - uptake feeds once ready, keeps running until button released.
 * 
 * Usage:
 * - Hold button to auto-aim and shoot
 * - Release button to stop everything
 */
public class AutoAimCommand extends Command {
    
    private final LimelightSubsystem m_limelight;
    private final HoodSubsystem m_hood;
    private final FlyWheelSubsystem m_shooter;
    private final UptakeSubsystem m_uptake;
    private final ShotCalculator m_calculator;
    
    // Constant shooter speed
    private static final double SHOOTER_SPEED = 0.7;  // Adjust this to your preferred speed
    
    // Tolerance for "ready to shoot" indicator
    private static final double HOOD_POSITION_TOLERANCE = 0.02;  // rotations (~7 degrees)
    private static final double ALIGNMENT_TOLERANCE = 3.0;  // degrees horizontal offset
    
    // Track if we've started feeding
    private boolean m_isFeedingStarted = false;
    
    public AutoAimCommand(
        LimelightSubsystem limelight,
        HoodSubsystem hood,
        FlyWheelSubsystem shooter,
        UptakeSubsystem uptake,
        ShotCalculator calculator
    ) {
        m_limelight = limelight;
        m_hood = hood;
        m_shooter = shooter;
        m_uptake = uptake;
        m_calculator = calculator;
        
        addRequirements(hood, shooter, uptake);
        // Note: NOT requiring limelight so it can still feed vision measurements
    }
    
    @Override
    public void initialize() {
        // Turn on Limelight LEDs for targeting
        m_limelight.setLEDsOn();
        
        // Reset feeding state
        m_isFeedingStarted = false;
        
        System.out.println("Auto-aim started (hold button - will feed when ready)");
    }
    
    @Override
    public void execute() {
        // Get current distance to target
        double distance = m_limelight.getDistanceToTargetSafe();
        
        // Calculate optimal hood angle
        double hoodAngle = m_calculator.calculateHoodAngle(distance);
        
        // Always adjust hood and run shooter while button is held
        m_hood.setPositionCommand(hoodAngle).schedule();
        m_shooter.shootAtSpeed(SHOOTER_SPEED).schedule();
        
        // Check if we're ready to shoot
        boolean hasTarget = m_limelight.hasValidTarget();
        boolean isAligned = Math.abs(m_limelight.getHorizontalOffset()) < ALIGNMENT_TOLERANCE;
        boolean hoodReady = Math.abs(m_hood.getPositionError()) < HOOD_POSITION_TOLERANCE;
        boolean validDistance = m_calculator.isValidDistance(distance);
        boolean optimalRange = m_calculator.isOptimalRange(distance);
        
        boolean readyToShoot = hasTarget && isAligned && hoodReady && validDistance;
        
        // Start feeding once ready, keep feeding while button held
        if (readyToShoot) {
            if (!m_isFeedingStarted) {
                System.out.println("READY - Starting uptake feed");
                m_isFeedingStarted = true;
            }
            m_uptake.runCommand().schedule();
        } else {
            // Not ready yet - stop uptake but keep preparing
            if (m_isFeedingStarted) {
                System.out.println("Lost ready state - stopping uptake");
                m_isFeedingStarted = false;
            }
            m_uptake.stopCommand().schedule();
        }
        
        // Log telemetry
        SmartDashboard.putNumber("AutoAim/Distance", distance);
        SmartDashboard.putNumber("AutoAim/HoodAngle_Target", hoodAngle);
        SmartDashboard.putNumber("AutoAim/HoodAngle_Actual", m_hood.getPosition());
        SmartDashboard.putNumber("AutoAim/ShooterSpeed", SHOOTER_SPEED);
        SmartDashboard.putBoolean("AutoAim/HasTarget", hasTarget);
        SmartDashboard.putBoolean("AutoAim/IsAligned", isAligned);
        SmartDashboard.putBoolean("AutoAim/HoodReady", hoodReady);
        SmartDashboard.putBoolean("AutoAim/ValidDistance", validDistance);
        SmartDashboard.putBoolean("AutoAim/OptimalRange", optimalRange);
        SmartDashboard.putBoolean("AutoAim/ReadyToShoot", readyToShoot);
        SmartDashboard.putBoolean("AutoAim/Feeding", m_isFeedingStarted);
        
        // Update status messages
        if (!readyToShoot) {
            if (!hasTarget) {
                SmartDashboard.putString("AutoAim/Status", "NO TARGET");
            } else if (!validDistance) {
                SmartDashboard.putString("AutoAim/Status", "OUT OF RANGE");
            } else if (!isAligned) {
                SmartDashboard.putString("AutoAim/Status", "NOT ALIGNED");
            } else if (!hoodReady) {
                SmartDashboard.putString("AutoAim/Status", "HOOD MOVING");
            }
        } else {
            SmartDashboard.putString("AutoAim/Status", "SHOOTING!");
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        // Turn off LEDs when done
        m_limelight.setLEDsOff();
        
        // Stop all shooter mechanisms
        m_shooter.stopCommand().schedule();
        m_uptake.stopCommand().schedule();
        m_hood.stowCommand().schedule();
        
        SmartDashboard.putString("AutoAim/Status", "IDLE");
        SmartDashboard.putBoolean("AutoAim/Feeding", false);
        
        System.out.println("Auto-aim ended: " + (interrupted ? "interrupted" : "button released"));
    }
    
    @Override
    public boolean isFinished() {
        // Run continuously while button is held
        return false;
    }
    
    /**
     * Checks if currently feeding (shooting).
     */
    public boolean isFeeding() {
        return m_isFeedingStarted;
    }
    
    /**
     * Checks if all conditions are met to take a shot.
     */
    public boolean isReadyToShoot() {
        boolean hasTarget = m_limelight.hasValidTarget();
        boolean isAligned = Math.abs(m_limelight.getHorizontalOffset()) < ALIGNMENT_TOLERANCE;
        boolean hoodReady = Math.abs(m_hood.getPositionError()) < HOOD_POSITION_TOLERANCE;
        double distance = m_limelight.getDistanceToTargetSafe();
        boolean validDistance = m_calculator.isValidDistance(distance);
        
        return hasTarget && isAligned && hoodReady && validDistance;
    }
}