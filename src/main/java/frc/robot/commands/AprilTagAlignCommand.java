package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.limelight.LimelightSubsystem;

public class AprilTagAlignCommand extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final LimelightSubsystem m_limelight;
    private final DoubleSupplier m_forwardSupplier;
    private final DoubleSupplier m_strafeYSupplier;
    private final double m_maxSpeed;
    private final double m_maxAngularRate;
    private final double m_speedMultiplier;
    
    private final SwerveRequest.RobotCentric m_robotCentricRequest;

    public AprilTagAlignCommand(
        CommandSwerveDrivetrain drivetrain,
        LimelightSubsystem limelight,
        DoubleSupplier forwardSupplier,
        DoubleSupplier strafeYSupplier,
        double maxSpeed,
        double maxAngularRate,
        double speedMultiplier
    ) {
        m_drivetrain = drivetrain;
        m_limelight = limelight;
        m_forwardSupplier = forwardSupplier;
        m_strafeYSupplier = strafeYSupplier;
        m_maxSpeed = maxSpeed;
        m_maxAngularRate = maxAngularRate;
        m_speedMultiplier = speedMultiplier;
        
        m_robotCentricRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);
        
        addRequirements(drivetrain, limelight);
    }
    
    @Override
    public void initialize() {
        // Turn on Limelight LEDs when tracking starts
        m_limelight.setLEDsOn();
    }
    
    @Override
    public void execute() {
        // Limelight only controls rotation
        double rotationalRate = m_limelight.calculateAimVelocity(m_maxAngularRate);
        
        // Manually control forward/backward with left stick Y
        double forwardSpeed = -MathUtil.applyDeadband(m_forwardSupplier.getAsDouble(), 0.1) 
            * m_maxSpeed * m_speedMultiplier;
        
        // Manually control strafing with left stick X
        double strafeSpeed = -MathUtil.applyDeadband(m_strafeYSupplier.getAsDouble(), 0.1) 
            * m_maxSpeed * m_speedMultiplier;
        
        // Apply robot-centric control
        m_drivetrain.setControl(
            m_robotCentricRequest
                .withVelocityX(forwardSpeed)
                .withVelocityY(strafeSpeed)
                .withRotationalRate(rotationalRate)
        );
    }
    
    @Override
    public void end(boolean interrupted) {
        // Turn off Limelight LEDs when tracking ends
        m_limelight.setLEDsOff();
        
        // Stop the drivetrain
        m_drivetrain.setControl(
            m_robotCentricRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
        );
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}