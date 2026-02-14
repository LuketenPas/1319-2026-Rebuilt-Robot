package frc.robot.commands.auto;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AlignAndShootCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.shooter.FlyWheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.UptakeSubsystem;

/**
 * Registers named commands for use in PathPlanner.
 * 
 * PathPlanner can trigger these commands using event markers in your .auto files.
 * This is MUCH better than trying to chain paths in code!
 */
public class AutoRoutines {
    
    private static final double DEFAULT_SHOOT_DURATION = 5.0;  // seconds
    private static final double DEFAULT_MAX_ANGULAR_RATE = 3.0;  // rad/s
    private static final double SHOOTER_SPEED = 0.7;
    
    /**
     * Registers all named commands for PathPlanner to use.
     * Call this once from RobotContainer constructor BEFORE building auto chooser.
     */
    public static void registerNamedCommands(
        CommandSwerveDrivetrain drivetrain,
        LimelightSubsystem limelight,
        HoodSubsystem hood,
        FlyWheelSubsystem shooter,
        UptakeSubsystem uptake,
        ShotCalculator calculator
    ) {
        // ===== SHOOTING COMMANDS =====
        
        // Full align and shoot sequence (5 seconds)
        NamedCommands.registerCommand(
            "AlignAndShoot",
            new AlignAndShootCommand(
                drivetrain, limelight, hood, shooter, uptake, calculator,
                DEFAULT_SHOOT_DURATION, DEFAULT_MAX_ANGULAR_RATE
            )
        );
        
        // Quick 3-second shot
        NamedCommands.registerCommand(
            "QuickShot",
            new AlignAndShootCommand(
                drivetrain, limelight, hood, shooter, uptake, calculator,
                3.0, DEFAULT_MAX_ANGULAR_RATE
            )
        );
        
        // Extended 7-second shot
        NamedCommands.registerCommand(
            "ExtendedShot",
            new AlignAndShootCommand(
                drivetrain, limelight, hood, shooter, uptake, calculator,
                7.0, DEFAULT_MAX_ANGULAR_RATE
            )
        );
        
        // ===== SHOOTER CONTROL COMMANDS =====
        
        // Prepare shooter (spin up and aim hood, but don't feed)
        NamedCommands.registerCommand(
            "PrepareShooter",
            Commands.sequence(
                Commands.runOnce(() -> {
                    double distance = limelight.getDistanceToTargetSafe();
                    double hoodAngle = calculator.calculateHoodAngle(distance);
                    hood.setPositionCommand(hoodAngle).schedule();
                    shooter.shootAtSpeed(SHOOTER_SPEED).schedule();
                    limelight.setLEDsOn();
                    System.out.println("Preparing shooter - Hood angle: " + hoodAngle);
                }),
                Commands.waitSeconds(0.5)  // Give hood time to move
            )
        );
        
        // Start feeding (assumes shooter already prepared)
        NamedCommands.registerCommand(
            "StartFeeding",
            Commands.runOnce(() -> {
                uptake.runCommand().schedule();
                System.out.println("Started feeding");
            })
        );
        
        // Stop feeding
        NamedCommands.registerCommand(
            "StopFeeding",
            Commands.runOnce(() -> {
                uptake.stopCommand().schedule();
                System.out.println("Stopped feeding");
            })
        );
        
        // Stop all shooter mechanisms
        NamedCommands.registerCommand(
            "StopShooter",
            Commands.runOnce(() -> {
                shooter.stopCommand().schedule();
                uptake.stopCommand().schedule();
                hood.stowCommand().schedule();
                limelight.setLEDsOff();
                System.out.println("Stopped shooter systems");
            })
        );
        
        // ===== HOOD COMMANDS =====
        
        // Aim hood based on current distance
        NamedCommands.registerCommand(
            "AimHood",
            Commands.runOnce(() -> {
                double distance = limelight.getDistanceToTargetSafe();
                double hoodAngle = calculator.calculateHoodAngle(distance);
                hood.setPositionCommand(hoodAngle).schedule();
                System.out.println("Aiming hood - Distance: " + distance + "m, Angle: " + hoodAngle);
            })
        );
        
        // Stow hood
        NamedCommands.registerCommand(
            "StowHood",
            hood.stowCommand()
        );
        
        // ===== LIMELIGHT COMMANDS =====
        
        // Turn on LEDs
        NamedCommands.registerCommand(
            "LimelightOn",
            Commands.runOnce(() -> limelight.setLEDsOn())
        );
        
        // Turn off LEDs
        NamedCommands.registerCommand(
            "LimelightOff",
            Commands.runOnce(() -> limelight.setLEDsOff())
        );
        
        // ===== UTILITY COMMANDS =====
        
        // Wait commands with different durations
        NamedCommands.registerCommand("Wait1", Commands.waitSeconds(1.0));
        NamedCommands.registerCommand("Wait2", Commands.waitSeconds(2.0));
        NamedCommands.registerCommand("Wait3", Commands.waitSeconds(3.0));
        
        // Print debug message
        NamedCommands.registerCommand(
            "PrintCheckpoint",
            Commands.runOnce(() -> 
                System.out.println("=== AUTO CHECKPOINT REACHED ===")
            )
        );
        
        System.out.println("Named commands registered for PathPlanner");
    }
    
    /**
     * Simple autonomous that just shoots from starting position.
     * Good for testing or when you just need to score preload.
     */
    public static Command shootOnlyAuto(
        CommandSwerveDrivetrain drivetrain,
        LimelightSubsystem limelight,
        HoodSubsystem hood,
        FlyWheelSubsystem shooter,
        UptakeSubsystem uptake,
        ShotCalculator calculator
    ) {
        return Commands.sequence(
            Commands.runOnce(() -> 
                System.out.println("=== SHOOT ONLY AUTO ===")
            ),
            
            new AlignAndShootCommand(
                drivetrain, limelight, hood, shooter, uptake, calculator,
                DEFAULT_SHOOT_DURATION, DEFAULT_MAX_ANGULAR_RATE
            ),
            
            Commands.runOnce(() -> 
                System.out.println("=== AUTO COMPLETE ===")
            )
        );
    }
}