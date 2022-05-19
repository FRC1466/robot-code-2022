package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoCommand extends CommandBase {
    
    private final DriveSubsystem m_drive;
    private double m_fwd = 0;
    private double m_rot = 0;

    public AutoCommand(DriveSubsystem subsystem, double fwd, double rot) {
        m_drive = subsystem;
        addRequirements(m_drive);

        m_fwd = fwd;
        m_rot = rot;
    }

    

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        m_drive.arcadeDriveAutoPID(m_fwd, m_rot);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return super.isFinished();
    }
}
