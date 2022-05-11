package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class ArmCommand extends CommandBase {
    private final IntakeSubsystem m_arm;
    private final XboxController m_controller;

    public ArmCommand(IntakeSubsystem subsystem, XboxController controller) {
        m_arm = subsystem;
        addRequirements(m_arm);
        m_controller = controller;
    }

    @Override
    public void execute() {
        double forward = m_controller.getRightY();
        m_arm.runArm(forward);

    }

}
