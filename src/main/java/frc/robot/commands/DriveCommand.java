package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import java.lang.Math;

public class DriveCommand extends CommandBase {
    private final DriveSubsystem m_drive;
    private final XboxController m_controller;
    
    public DriveCommand(DriveSubsystem subsystem, XboxController controller) {
        m_drive = subsystem;
        addRequirements(m_drive);
        m_controller = controller;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        double pos = m_controller.getRightTriggerAxis();
        double neg = m_controller.getLeftTriggerAxis();
        double forward = pos - neg;
        double rot = Math.pow(m_controller.getLeftX(), 3);
        m_drive.arcadeDrive(-forward, rot);
    }
}
