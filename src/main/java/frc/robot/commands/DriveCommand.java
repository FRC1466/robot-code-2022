package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {
    private final DriveSubsystem m_drive;
    private final double m_forward;
    private final double m_rotation;
    
    public DriveCommand(DriveSubsystem subsystem, double forward, double rotation) {
        m_drive = subsystem;
        m_forward = forward;
        m_rotation = rotation;
    }

    public void initialize() {
        
    }
    public void execute() {
        m_drive.arcadeDrive(m_forward, m_rotation);
    }
}
