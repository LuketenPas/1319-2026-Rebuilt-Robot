package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Controls the dual-motor flywheel shooter.
 *
 * Uses a piecewise kP model based on empirical calibration data:
 *   0.5 m → 45 RPS → kP = 0.50
 *   1.0 m → 48 RPS → kP = 0.50
 *   2.0 m → 70 RPS → kP = 1.20
 *
 * Velocity model: v(d) = 43 - 2d + 8d²
 * kP model: constant 0.5 below 50 RPS, scales as 0.5×(v/50)^2.4 above 50 RPS
 */
public class FlyWheelSubsystem extends SubsystemBase {
    // Hardware
    private final TalonFX m_shooterMotor1;
    private final TalonFX m_shooterMotor2;
    private final VelocityVoltage m_velocityRequest;

    // CAN IDs
    private static final int    kMotor1Id = 11;
    private static final int    kMotor2Id = 12;
    private static final String kCanBus   = "canivore";

    // Piecewise kP constants
    private static final double kPTransitionVelocity = 50.0;
    private static final double kPBase               = 0.5;
    private static final double kPVelocityExponent   = 2.4;
    private static final double kReferenceVelocity   = 50.0;

    // Fixed feed-forward gains
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kS = 0.2;
    private static final double kV = 0.133;
    private static final double kA = 0.01;

    // Distance-to-velocity quadratic model coefficients: v = a + b*d + c*d²
    private static final double kVelocityBase      = 43.0;
    private static final double kVelocityLinear    = -2.0;
    private static final double kVelocityQuadratic = 8.0;

    // Velocity limits
    private static final double kMinVelocityRps = 30.0;
    private static final double kMaxVelocityRps = 75.0;

    // At-speed tolerance and safety timeout
    private static final double kVelocityToleranceRps = 2.0;
    private static final double kAtSpeedTimeoutSeconds = 3.0;

    // State
    private double m_targetVelocity = 0.0;

    public FlyWheelSubsystem() {
        m_shooterMotor1 = new TalonFX(kMotor1Id, kCanBus);
        m_shooterMotor2 = new TalonFX(kMotor2Id, kCanBus);
        m_velocityRequest = new VelocityVoltage(0).withSlot(0);

        configureMotors();
    }

    private void configureMotors() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = kPBase;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        m_shooterMotor1.getConfigurator().apply(config);
        m_shooterMotor2.setControl(new Follower(kMotor1Id, MotorAlignmentValue.Opposed));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/Velocity",       getCurrentVelocity());
        SmartDashboard.putNumber("Shooter/TargetVelocity", m_targetVelocity);
        SmartDashboard.putBoolean("Shooter/AtSpeed",       isAtSpeed());
    }

    // ========== Getters ==========

    public double getCurrentVelocity() {
        return m_shooterMotor1.getVelocity().getValueAsDouble();
    }

    public boolean isAtSpeed() {
        return Math.abs(m_targetVelocity - getCurrentVelocity()) < kVelocityToleranceRps;
    }

    // ========== Private Helpers ==========

    /**
     * Calculates scaled kP using a piecewise model.
     * Below 50 RPS: constant kP = 0.5.
     * Above 50 RPS: kP = 0.5 × (v/50)^2.4.
     */
    private double calculateScaledKp(double velocityRps) {
        if (velocityRps <= kPTransitionVelocity) {
            return kPBase;
        }

        double scaledKp = kPBase * Math.pow(velocityRps / kReferenceVelocity, kPVelocityExponent);
        return Math.max(0.1, Math.min(5.0, scaledKp));
    }

    /** Applies updated PID gains to the motor controller. */
    private void updatePidGains(double kp) {
        Slot0Configs slot0 = new Slot0Configs();
        slot0.kP = kp;
        slot0.kI = kI;
        slot0.kD = kD;
        slot0.kS = kS;
        slot0.kV = kV;
        slot0.kA = kA;

        m_shooterMotor1.getConfigurator().apply(slot0);
    }

    /**
     * Calculates the target flywheel velocity for a given distance.
     * Uses quadratic model: v = 43 - 2d + 8d²
     */
    private double calculateVelocityForDistance(double distanceMeters) {
        double velocity = kVelocityBase
            + kVelocityLinear    * distanceMeters
            + kVelocityQuadratic * distanceMeters * distanceMeters;

        return Math.max(kMinVelocityRps, Math.min(kMaxVelocityRps, velocity));
    }

    /** Sets flywheel velocity and updates kP scaling accordingly. */
    private void setVelocity(double velocityRps) {
        m_targetVelocity = velocityRps;
        updatePidGains(calculateScaledKp(velocityRps));
        m_shooterMotor1.setControl(m_velocityRequest.withVelocity(velocityRps));
    }

    /** Sets flywheel velocity based on a measured distance. */
    private void setVelocityForDistance(double distanceMeters) {
        setVelocity(calculateVelocityForDistance(distanceMeters));
    }

    private void stop() {
        m_targetVelocity = 0.0;
        m_shooterMotor1.set(0);
    }

    // ========== Commands ==========

    /** Spins the flywheel to a specific velocity in RPS. kP scales automatically. */
    public Command shootAtVelocity(double velocityRps) {
        return runOnce(() -> setVelocity(velocityRps));
    }

    /** Spins the flywheel to the correct velocity for the given distance in meters. */
    public Command shootAtDistance(double distanceMeters) {
        return runOnce(() -> setVelocityForDistance(distanceMeters));
    }

    /** Stops the flywheel. */
    public Command stopCommand() {
        return runOnce(this::stop);
    }

    /**
     * Waits until the flywheel reaches target speed.
     * Times out after {@value kAtSpeedTimeoutSeconds} seconds as a safety measure.
     */
    public Command waitUntilAtSpeed() {
        return run(() -> {}).until(this::isAtSpeed).withTimeout(kAtSpeedTimeoutSeconds);
    }
}