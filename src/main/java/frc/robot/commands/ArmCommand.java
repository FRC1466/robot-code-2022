package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import java.lang.Math;

public class ArmCommand extends CommandBase {
    private final IntakeSubsystem m_arm;
    private final XboxController m_controller;
    private boolean isMoving;

    public ArmCommand(IntakeSubsystem subsystem, XboxController controller) {
        m_arm = subsystem;
        addRequirements(m_arm);
        m_controller = controller;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double forward = m_controller.getRightY();
        if (Math.abs(forward) < 0.10) {
            forward = 0;
            isMoving = false;
        } else {
            isMoving = true;
        }
        m_arm.runArm(forward, isMoving);

    }

}
