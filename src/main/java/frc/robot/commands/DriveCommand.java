package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

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
        m_drive.arcadeDrive(m_controller.getLeftY(), m_controller.getLeftX());
    }
}
