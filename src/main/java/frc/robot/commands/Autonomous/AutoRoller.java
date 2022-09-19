package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoRoller extends CommandBase {
    private final IntakeSubsystem m_intake;
    private final DriveSubsystem m_drive;
    private boolean m_isRoll;


    public AutoRoller(IntakeSubsystem intake, DriveSubsystem drive, boolean isRoll) {
        m_intake = intake;
        m_drive = drive;
        addRequirements(m_intake);
        addRequirements(m_drive);
        m_isRoll = isRoll;
    }

    

    @Override
    public void execute() {
        
        m_drive.pacifyDrive();
        if(m_isRoll) {
            m_intake.runRoller(-1.6);
        } else {
            m_intake.stopRoller();
        }
        
    }
}
