// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {
    //Hardware
    private final TalonFX hoodMotor;
    private final MotionMagicVoltage motionMagicRequest;

    //Constants
    private static final int HOOD_MOTOR_ID = 13;
    private static final String CAN_BUS = "canivore";
    
    //Gear ratio
    private static final double GEAR_RATIO = 90.0;
    
    //Motion Magic parameters
    private static final double MM_CRUISE_VELOCITY = 5.0;  //rotations per second
    private static final double MM_ACCELERATION = 10.0;     //rotations per second^2
    private static final double MM_JERK = 100.0;            //rotations per second^3
    
    //PID gains
    private static final double kS = 0.25;  //Static friction feedforward (V)
    private static final double kV = 0.12;  //Velocity feedforward (V per rps)
    private static final double kA = 0.01;  //Acceleration feedforward (V per rps^2)
    private static final double kP = 60.0;  //Proportional gain
    private static final double kI = 0.0;   //Integral gain
    private static final double kD = 0.5;   //Derivative gain
    
    //Position limits
    private static final double MIN_POSITION = 0.0;    //rotations (horizontal/stowed - at limit switch)
    private static final double MAX_POSITION = 0.25;   //rotations (max angle)
    
    //Preset positions for common shots
    private static final double STOW_POSITION = 0.0;       //Stowed/safe position
    private static final double CLOSE_SHOT_POSITION = 0.1; //Close range shot
    private static final double FAR_SHOT_POSITION = 0.2;   //Far range shot
    
    //Limit switch tracking
    private boolean wasLimitSwitchPressed = false;

    public HoodSubsystem() {
        hoodMotor = new TalonFX(HOOD_MOTOR_ID, CAN_BUS);
        motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);

        configureMotor();
    }

    //Configures the hood motor with Motion Magic parameters, PID gains, and limit switch
    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        //Configure feedback (gear ratio)
        FeedbackConfigs feedbackConfigs = config.Feedback;
        feedbackConfigs.SensorToMechanismRatio = GEAR_RATIO;

        //Configure Motion Magic profile
        MotionMagicConfigs motionMagicConfigs = config.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = MM_CRUISE_VELOCITY;
        motionMagicConfigs.MotionMagicAcceleration = MM_ACCELERATION;
        motionMagicConfigs.MotionMagicJerk = MM_JERK;

        //Configure PID and feedforward gains
        Slot0Configs slot0Configs = config.Slot0;
        slot0Configs.kS = kS;
        slot0Configs.kV = kV;
        slot0Configs.kA = kA;
        slot0Configs.kP = kP;
        slot0Configs.kI = kI;
        slot0Configs.kD = kD;

        //Set motor to brake mode
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        //Configure limit switches
        HardwareLimitSwitchConfigs limitSwitchConfigs = config.HardwareLimitSwitch;
        
        //Configure REVERSE limit switch
        limitSwitchConfigs.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
        limitSwitchConfigs.ReverseLimitEnable = true;  //Enable the limit switch
        limitSwitchConfigs.ReverseLimitAutosetPositionEnable = true;  //Auto-reset position when hit
        limitSwitchConfigs.ReverseLimitAutosetPositionValue = MIN_POSITION;  //Reset to 0.0 rotations

        //Apply configuration with retry logic
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++) {
            status = hoodMotor.getConfigurator().apply(config);
            if (status.isOK()) {
                System.out.println("Hood motor configured successfully with limit switch zeroing");
                break;
            }
        }
        
        if (!status.isOK()) {
            System.err.println("Failed to configure hood motor. Error: " + status.toString());
        }

        // Set initial position to 0 (assumes starting at limit switch)
        hoodMotor.setPosition(STOW_POSITION);
    }

    @Override
    public void periodic() {
        //Check limit switch status
        boolean isReverseLimitPressed = getReverseLimitSwitch();
        
        //Detect limit switch press
        if (isReverseLimitPressed && !wasLimitSwitchPressed) {
            System.out.println("Hood limit switch hit - position reset to " + MIN_POSITION);
        }
        
        wasLimitSwitchPressed = isReverseLimitPressed;
        
        //Log hood position and status to dashboard
        SmartDashboard.putNumber("Hood/Position_Rotations", getPosition());
        SmartDashboard.putNumber("Hood/Position_Degrees", getPosition() * 360);
        SmartDashboard.putNumber("Hood/Velocity_RPS", getVelocity());
        SmartDashboard.putNumber("Hood/Target_Position", motionMagicRequest.Position);
        SmartDashboard.putBoolean("Hood/At_Target", isAtTarget());
        SmartDashboard.putNumber("Hood/Error_Rotations", getPositionError());
        SmartDashboard.putBoolean("Hood/Reverse_Limit_Switch", isReverseLimitPressed);
    }

    //Getters

    //Gets the current hood position
    public double getPosition() {
        return hoodMotor.getPosition().getValueAsDouble();
    }

    //Gets the current hood velocity
    public double getVelocity() {
        return hoodMotor.getVelocity().getValueAsDouble();
    }

    //Gets the position error
    public double getPositionError() {
        return motionMagicRequest.Position - getPosition();
    }

    //Checks if hood is at target position
    public boolean isAtTarget() {
        return Math.abs(getPositionError()) < 0.01; // Within 0.01 rotations (~3.6 degrees)
    }

    //Gets the reverse limit switch status
    public boolean getReverseLimitSwitch() {
        return hoodMotor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
    }

    //Private Helper Methods

    //Sets the hood to a specific position using Motion Magic
    private void setPosition(double rotations) {
        //Clamp to safe limits
        double clampedPosition = Math.max(MIN_POSITION, Math.min(MAX_POSITION, rotations));
        hoodMotor.setControl(motionMagicRequest.withPosition(clampedPosition));
    }

    //Runs the hood motor at a manual speed
    private void setManualSpeed(double speed) {
        hoodMotor.set(speed);
    }

    // Stops the hood motor
    private void stop() {
        hoodMotor.set(0);
    }

    //Manually resets the hood position to the stow position
    private void resetPosition() {
        hoodMotor.setPosition(STOW_POSITION);
    }

    //Public Commands

    //Command to set hood to a specific position
    public Command setPositionCommand(double rotations) {
        return runOnce(() -> setPosition(rotations));
    }

    //Command to move hood to stow position (safe position at limit switch)
    public Command stowCommand() {
        return runOnce(() -> setPosition(STOW_POSITION));
    }

    //Command to set hood for close range shot
    public Command closeShotCommand() {
        return runOnce(() -> setPosition(CLOSE_SHOT_POSITION));
    }

    //Command to set hood for far range shot
    public Command farShotCommand() {
        return runOnce(() -> setPosition(FAR_SHOT_POSITION));
    }

    //Command for manual hood control with joystick
    public Command manualControlCommand(DoubleSupplier speedSupplier) {
        return run(() -> setManualSpeed(speedSupplier.getAsDouble()));
    }

    //Command to stop the hood motor
    public Command stopCommand() {
        return runOnce(() -> stop());
    }

    //Command to manually reset the hood position to stow
    public Command resetPositionCommand() {
        return runOnce(() -> resetPosition());
    }

    //Command to home the hood by slowly moving to limit switch
    public Command homeCommand() {
        return run(() -> {
            if (!getReverseLimitSwitch()) {
                // Move slowly toward limit switch
                setManualSpeed(-0.3);
            } else {
                // Stop when limit switch is hit (position auto-resets)
                stop();
            }
        }).until(this::getReverseLimitSwitch)
          .andThen(stopCommand());
    }
}