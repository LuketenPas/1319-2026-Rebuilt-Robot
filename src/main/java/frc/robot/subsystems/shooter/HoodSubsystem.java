package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Controls the hood pneumatic cylinder to toggle the shooter angle. */
public class HoodSubsystem extends SubsystemBase {
    private final DoubleSolenoid m_hoodSolenoid;

    // Pneumatic channel IDs on the REV PH
    private static final int kSolenoidForwardChannel = 4;
    private static final int kSolenoidReverseChannel = 5;

    public HoodSubsystem() {
        m_hoodSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, kSolenoidForwardChannel, kSolenoidReverseChannel);
        m_hoodSolenoid.set(Value.kReverse);
    }

    @Override
    public void periodic() {}

    /** Returns true if the hood is in the raised (far shot) position. */
    public boolean isRaised() {
        return m_hoodSolenoid.get() == Value.kForward;
    }

    /** Toggles the hood between raised and lowered positions. */
    public Command toggleCommand() {
        return Commands.runOnce(() -> m_hoodSolenoid.toggle(), this);
    }

    /** Raises the hood for far shots. */
    public Command raiseCommand() {
        return Commands.runOnce(() -> m_hoodSolenoid.set(Value.kForward), this);
    }

    /** Lowers the hood for close shots. */
    public Command lowerCommand() {
        return Commands.runOnce(() -> m_hoodSolenoid.set(Value.kReverse), this);
    }
}
