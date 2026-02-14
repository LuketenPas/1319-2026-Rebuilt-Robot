// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AprilTagAlignCommand;
import frc.robot.commands.ShootingCommands;
import frc.robot.commands.auto.AutoRoutines;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.shooter.FlyWheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.shooter.UptakeSubsystem;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // Speed multipliers
    private final double[] speedMultiplier = {0.7};
    private final double SLOW_SPEED = 0.5;
    private final double NORMAL_SPEED = 0.7;
    private final double TURBO_SPEED = 1;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Controllers
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    //Subsystems
    // PRO FEATURE: Drivetrain now uses 250Hz odometry with FOC
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
    public final FlyWheelSubsystem flyWheelSubsystem = new FlyWheelSubsystem();
    public final HoodSubsystem hoodSubsystem = new HoodSubsystem();
    public final UptakeSubsystem uptakeSubsystem = new UptakeSubsystem();
    
    // NEW: Shot calculator for distance-based shooting
    public final ShotCalculator shotCalculator = new ShotCalculator();

    //Path follower - NOT final so we can initialize it in constructor
    private SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // PRO FEATURE: Connect limelight to drivetrain for vision measurement integration
        limelightSubsystem.setDrivetrain(drivetrain);
        
        // Register named commands FIRST (before building auto chooser)
        AutoRoutines.registerNamedCommands(
            drivetrain,
            limelightSubsystem,
            hoodSubsystem,
            flyWheelSubsystem,
            uptakeSubsystem,
            shotCalculator
        );
        
        // Build auto chooser - PathPlanner will find your .auto files
        configureAutoChooser();

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();

        // Home the hood on robot startup
        hoodSubsystem.homeCommand().schedule();
    }
    
    /**
     * Configures the autonomous chooser.
     * PathPlanner will automatically find .auto files in deploy/pathplanner/autos/
     */
    private void configureAutoChooser() {
        // Use PathPlanner's auto chooser - it finds all your .auto files automatically!
        autoChooser = AutoBuilder.buildAutoChooser();
        
        // Add a simple "Shoot Only" option for testing
        autoChooser.addOption(
            "Shoot Only (No Drive)",
            AutoRoutines.shootOnlyAuto(
                drivetrain,
                limelightSubsystem,
                hoodSubsystem,
                flyWheelSubsystem,
                uptakeSubsystem,
                shotCalculator
            )
        );
        
        SmartDashboard.putData("Auto Mode", autoChooser);
        
        System.out.println("Auto chooser configured - Named commands registered for PathPlanner");
        System.out.println("Create .auto files in PathPlanner GUI to build autonomous routines!");
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * MaxSpeed * speedMultiplier[0]) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed * speedMultiplier[0]) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate * speedMultiplier[0]) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Speed Control
        driverController.leftTrigger().whileTrue(
            Commands.runOnce(() -> speedMultiplier[0] = SLOW_SPEED)
        ).onFalse(
            Commands.runOnce(() -> speedMultiplier[0] = NORMAL_SPEED)
        );
        driverController.rightTrigger().whileTrue(
            Commands.runOnce(() -> speedMultiplier[0] = TURBO_SPEED)
        ).onFalse(
            Commands.runOnce(() -> speedMultiplier[0] = NORMAL_SPEED)
        );

        //Brake mode
        driverController.b().whileTrue(drivetrain.applyRequest(() -> brake));
        //Points wheels in controller direction
        driverController.x().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        ));

        //Drives forwards
        driverController.povUp().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        //Drives backwards
        driverController.povDown().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // IMPORTANT: Re-run SysId after enabling FOC - the gains will be different!
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        driverController.a().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        //Tracks AprilTag (rotation only - driver still controls translation)
        driverController.rightBumper().whileTrue(
            new AprilTagAlignCommand(
                drivetrain,
                limelightSubsystem,
                () -> driverController.getLeftY(),
                () -> driverController.getLeftX(),
                MaxSpeed,
                MaxAngularRate,
                speedMultiplier[0]
            )
        );

        // ========== OPERATOR CONTROLS ==========
        
        // AUTO-AIM AND SHOOT - HOLD BUTTON
        // Right trigger: Hold to auto-aim and shoot
        // - Spins up shooter and adjusts hood
        // - Once ready, starts feeding
        // - Keeps shooting while held
        // - Release to stop everything
        operatorController.rightTrigger().whileTrue(
            ShootingCommands.autoAimAndShoot(
                limelightSubsystem,
                hoodSubsystem,
                flyWheelSubsystem,
                uptakeSubsystem,
                shotCalculator
            )
        );
        
        // MANUAL SHOOT - HOLD BUTTON (backup mode)
        // A button: Manual shooting with fixed speed
        // Hold to shoot, release to stop
        operatorController.a().whileTrue(
            flyWheelSubsystem.shootCommand()
            .alongWith(Commands.waitSeconds(1).andThen(uptakeSubsystem.runCommand()))
        );
        operatorController.a().onFalse(
            flyWheelSubsystem.stopCommand()
            .alongWith(uptakeSubsystem.stopCommand())
        );

        // Hood manual control (for fine-tuning during testing)
        // Left stick Y always controls hood manually
        hoodSubsystem.setDefaultCommand(
            hoodSubsystem.manualControlCommand(() -> -operatorController.getLeftY() * 0.1)
        );

        // Hood preset positions (quick overrides during match)
        operatorController.povUp().onTrue(hoodSubsystem.farShotCommand());     // Far shot preset
        operatorController.povLeft().onTrue(hoodSubsystem.closeShotCommand()); // Close shot preset
        operatorController.povDown().onTrue(hoodSubsystem.stowCommand());      // Stow position

        // Manual hood home (if limit switch is triggered)
        operatorController.back().onTrue(hoodSubsystem.homeCommand());

        // Manual position reset (only use when AT the limit switch)
        operatorController.start().onTrue(hoodSubsystem.resetPositionCommand());
        
        // B button: Emergency stop all shooter mechanisms
        operatorController.b().onTrue(
            Commands.parallel(
                flyWheelSubsystem.stopCommand(),
                uptakeSubsystem.stopCommand(),
                hoodSubsystem.stopCommand()
            ).andThen(Commands.runOnce(() -> 
                System.out.println("EMERGENCY STOP - All shooter mechanisms stopped")
            ))
        );
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}