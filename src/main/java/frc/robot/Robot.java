package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    UsbCamera frontCamera = CameraServer.startAutomaticCapture("Front Camera", 0);
    frontCamera.setResolution(320, 240);
    frontCamera.setFPS(15);
    UsbCamera backCamera = CameraServer.startAutomaticCapture("Back Camera", 1);
    backCamera.setResolution(320, 240);
    backCamera.setFPS(15);

    SignalLogger.start();
    System.out.println("Phoenix Pro Signal Logging Started - Logs will be saved to ./logs/swerve.hoot");
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    // Set Limelight to teleop mode when disabled
    m_robotContainer.limelightSubsystem.setAutoMode(false);
    m_robotContainer.limelightSubsystem.setLEDsOff();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    // PRO FEATURE: Enable vision corrections for auto
    m_robotContainer.limelightSubsystem.setAutoMode(true);
    m_robotContainer.limelightSubsystem.setVisionUpdatesEnabled(true);
    
    System.out.println("=== AUTONOMOUS STARTED ===");
    System.out.println("Vision odometry fusion: ENABLED");
    System.out.println("Limelight LEDs: ON for AprilTag tracking");
    
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // PRO FEATURE: Switch to teleop mode (less strict vision criteria)
    m_robotContainer.limelightSubsystem.setAutoMode(false);
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}