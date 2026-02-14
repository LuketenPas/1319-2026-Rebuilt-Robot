package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.shooter.FlyWheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.UptakeSubsystem;

/**
 * Factory class for creating shooting commands.
 * Shooter speed is constant - only hood angle adjusts based on distance.
 */
public class ShootingCommands {
    
    /**
     * Creates the main auto-aim and shoot command.
     * 
     * HOLD BUTTON behavior:
     * 1. Spins up shooter to constant speed
     * 2. Adjusts hood angle based on distance
     * 3. Once aligned and ready, starts feeding with uptake
     * 4. Keeps shooting as long as button is held
     * 
     * RELEASE BUTTON:
     * - Stops shooter, uptake, and returns hood to stow
     * 
     * @param limelight Limelight subsystem for distance measurement
     * @param hood Hood subsystem for angle adjustment
     * @param shooter Flywheel shooter subsystem
     * @param uptake Uptake subsystem to feed game piece
     * @param calculator Shot calculator for distance-based hood angle
     * @return Auto-aim and shoot command
     */
    public static Command autoAimAndShoot(
        LimelightSubsystem limelight,
        HoodSubsystem hood,
        FlyWheelSubsystem shooter,
        UptakeSubsystem uptake,
        ShotCalculator calculator
    ) {
        return new AutoAimCommand(limelight, hood, shooter, uptake, calculator);
    }
}