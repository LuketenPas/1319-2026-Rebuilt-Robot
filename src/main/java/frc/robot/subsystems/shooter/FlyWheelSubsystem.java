// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlyWheelSubsystem extends SubsystemBase {
  // Motors
  private final TalonFX shooterMotor1;
  private final TalonFX shooterMotor2;
  
  // Constants
  private static final int MOTOR_1_ID = 11;
  private static final int MOTOR_2_ID = 12;
  private static final String CAN_BUS = "canivore";
  private static final double SHOOTER_SPEED = 0.7;

  public FlyWheelSubsystem() {
    shooterMotor1 = new TalonFX(MOTOR_1_ID, CAN_BUS);
    shooterMotor2 = new TalonFX(MOTOR_2_ID, CAN_BUS);
    
    // Configure motor 1 to coast when neutral (spins down naturally)
    shooterMotor1.setNeutralMode(NeutralModeValue.Coast);
    shooterMotor2.setNeutralMode(NeutralModeValue.Coast);
    
    // Make motor 2 follow motor 1 with opposite direction
    shooterMotor2.setControl(new Follower(MOTOR_1_ID, MotorAlignmentValue.Opposed));
  }

  @Override
  public void periodic() {
  }

  // Helper Methods
  
  // Sets the shooter speed
  private void setShooterSpeed(double speed) {
    shooterMotor1.set(speed);
  }

  // Stops both shooter motors
  private void stop() {
    shooterMotor1.set(0);
  }

  // Public Commands
  
  // Command to run shooter wheels at default speed

  public Command shootCommand() {
    return runOnce(() -> setShooterSpeed(SHOOTER_SPEED));
  }

  // Command to stop the shooter wheels
  public Command stopCommand() {
    return runOnce(() -> stop());
  }
  
  // Command to run shooter at a custom speed
  public Command shootAtSpeed(double speed) {
    return runOnce(() -> setShooterSpeed(speed));
  }
}