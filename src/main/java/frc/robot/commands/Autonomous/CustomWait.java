package frc.robot.commands.Autonomous;

import javax.swing.plaf.multi.MultiListUI;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;

public class CustomWait extends CommandBase {
    private int iteration = 0;
    private final DriveSubsystem m_drive;
    private int m_milliseconds;

    CustomWait(DriveSubsystem drive, int milliseconds) {
        m_drive = drive;
        m_milliseconds = milliseconds;
    }

    @Override
    public void execute() {
        iteration++;
        m_drive.pacifyDrive();

    }

    @Override
    public boolean isFinished() {
        if (iteration*20 > m_milliseconds) {
            return true;
        } else {
            return false;
        }

    }
}
