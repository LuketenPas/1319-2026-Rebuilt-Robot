package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.shooter.FlyWheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.UptakeSubsystem;

/**
 * Autonomous command that:
 * 1. Rotates robot to align with AprilTag target
 * 2. Adjusts hood angle based on distance
 * 3. Spins up shooter
 * 4. Once aligned, shoots for a specified duration
 * 5. Stops all shooting mechanisms when done
 * 
 * This command is designed for autonomous shooting sequences.
 */
public class AlignAndShootCommand extends Command {
    
    private final CommandSwerveDrivetrain m_drivetrain;
    private final LimelightSubsystem m_limelight;
    private final HoodSubsystem m_hood;
    private final FlyWheelSubsystem m_shooter;
    private final UptakeSubsystem m_uptake;
    private final ShotCalculator m_calculator;
    
    private final double m_shootDuration;  // How long to shoot (seconds)
    private final double m_maxAngularRate;
    
    // Constant shooter speed
    private static final double SHOOTER_SPEED = 0.7;
    
    // Proportional control for rotation
    private static final double kP_ROTATION = 0.035;
    
    // Alignment tolerances
    private static final double ALIGNMENT_TOLERANCE = 2.0;  // degrees (tighter for auto)
    private static final double HOOD_POSITION_TOLERANCE = 0.015;  // rotations (tighter for auto)
    
    // State machine
    private enum ShootState {
        ALIGNING,    // Rotating to target and adjusting hood
        SHOOTING,    // Feeding game pieces
        DONE         // Finished
    }
    
    private ShootState m_state;
    private final Timer m_shootTimer;
    private final SwerveRequest.RobotCentric m_rotateRequest;
    
    /**
     * Creates a new AlignAndShootCommand.
     * 
     * @param drivetrain The swerve drivetrain
     * @param limelight The limelight subsystem
     * @param hood The hood subsystem
     * @param shooter The flywheel shooter subsystem
     * @param uptake The uptake subsystem
     * @param calculator The shot calculator
     * @param shootDuration How long to shoot in seconds (typically 3-5 seconds)
     * @param maxAngularRate Maximum rotation speed (radians/sec)
     */
    public AlignAndShootCommand(
        CommandSwerveDrivetrain drivetrain,
        LimelightSubsystem limelight,
        HoodSubsystem hood,
        FlyWheelSubsystem shooter,
        UptakeSubsystem uptake,
        ShotCalculator calculator,
        double shootDuration,
        double maxAngularRate
    ) {
        m_drivetrain = drivetrain;
        m_limelight = limelight;
        m_hood = hood;
        m_shooter = shooter;
        m_uptake = uptake;
        m_calculator = calculator;
        m_shootDuration = shootDuration;
        m_maxAngularRate = maxAngularRate;
        
        m_shootTimer = new Timer();
        m_rotateRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);
        
        addRequirements(drivetrain, hood, shooter, uptake);
    }
    
    @Override
    public void initialize() {
        m_state = ShootState.ALIGNING;
        m_shootTimer.reset();
        
        // Turn on Limelight LEDs
        m_limelight.setLEDsOn();
        
        System.out.println("=== ALIGN AND SHOOT AUTONOMOUS STARTED ===");
        System.out.println("Shoot duration: " + m_shootDuration + " seconds");
    }
    
    @Override
    public void execute() {
        // Always adjust hood and spin up shooter while command is running
        double distance = m_limelight.getDistanceToTargetSafe();
        double hoodAngle = m_calculator.calculateHoodAngle(distance);
        
        m_hood.setPositionCommand(hoodAngle).schedule();
        m_shooter.shootAtSpeed(SHOOTER_SPEED).schedule();
        
        switch (m_state) {
            case ALIGNING:
                executeAligning();
                break;
                
            case SHOOTING:
                executeShooting();
                break;
                
            case DONE:
                // Do nothing, wait for isFinished() to end command
                break;
        }
    }
    
    /**
     * ALIGNING state: Rotate robot to face target and wait for hood to settle
     */
    private void executeAligning() {
        // Check if we have a valid target
        if (!m_limelight.hasValidTarget()) {
            System.out.println("WARNING: No target visible during align!");
            // Stop rotating but keep preparing
            m_drivetrain.setControl(m_rotateRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
            );
            return;
        }
        
        // Calculate rotation velocity to aim at target
        double tx = m_limelight.getHorizontalOffset();
        double rotationalRate = tx * kP_ROTATION * m_maxAngularRate * -1.0;
        
        // Apply rotation (no translation during alignment)
        m_drivetrain.setControl(m_rotateRequest
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(rotationalRate)
        );
        
        // Check if aligned and ready to shoot
        boolean isAligned = Math.abs(tx) < ALIGNMENT_TOLERANCE;
        boolean hoodReady = Math.abs(m_hood.getPositionError()) < HOOD_POSITION_TOLERANCE;
        boolean validDistance = m_calculator.isValidDistance(m_limelight.getDistanceToTargetSafe());
        
        if (isAligned && hoodReady && validDistance) {
            // Transition to shooting
            m_state = ShootState.SHOOTING;
            m_shootTimer.start();
            System.out.println("ALIGNED - Starting to shoot!");
        }
    }
    
    /**
     * SHOOTING state: Keep robot still and feed game pieces
     */
    private void executeShooting() {
        // Stop robot movement during shooting
        m_drivetrain.setControl(m_rotateRequest
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0)
        );
        
        // Run uptake to feed game pieces
        m_uptake.runCommand().schedule();
        
        // Check if shoot duration has elapsed
        if (m_shootTimer.hasElapsed(m_shootDuration)) {
            m_state = ShootState.DONE;
            System.out.println("Shooting complete - " + m_shootDuration + " seconds elapsed");
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop all mechanisms
        m_drivetrain.setControl(m_rotateRequest
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0)
        );
        
        m_shooter.stopCommand().schedule();
        m_uptake.stopCommand().schedule();
        m_hood.stowCommand().schedule();
        
        // Turn off Limelight LEDs
        m_limelight.setLEDsOff();
        
        m_shootTimer.stop();
        
        System.out.println("=== ALIGN AND SHOOT COMPLETE ===");
        if (interrupted) {
            System.out.println("WARNING: Command was interrupted!");
        }
    }
    
    @Override
    public boolean isFinished() {
        return m_state == ShootState.DONE;
    }
    
    /**
     * Gets the current state (for debugging/telemetry).
     */
    public String getStateString() {
        return m_state.toString();
    }
}