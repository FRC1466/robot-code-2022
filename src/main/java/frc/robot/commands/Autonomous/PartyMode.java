package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

public class PartyMode extends CommandBase {
    private final DriveSubsystem m_drive;
    private int iter;
    private double seconds = 1;

    public PartyMode(DriveSubsystem drive) {
        m_drive = drive;
    }

    @Override
    public void execute() {
        if(iter < seconds*1000/20) {
            m_drive.arcadeDrive(0, AutoConstants.kPartyRot);
        } else {
            m_drive.arcadeDrive(0, -AutoConstants.kPartyRot);
        }
        iter++;
    }
}
