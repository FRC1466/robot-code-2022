package frc.robot.commands.Autonomous;

import javax.swing.plaf.multi.MultiListUI;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;


public class CustomWaitMillis extends CommandBase {
    private final DriveSubsystem m_drive;
    private int m_milliseconds;
    private long startTime;
    private long endTime;
    
    /**
     * Waits for some number of ms while calling DifferentialDrive.
     * @param drive drive subsystem
     * @param milliseconds waiting time in milliseconds
     */
    CustomWaitMillis(DriveSubsystem drive, int milliseconds) {
        m_drive = drive;
        m_milliseconds = milliseconds;

        
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        endTime = startTime + m_milliseconds;
    }

    @Override
    public void execute() {
        m_drive.pacifyDrive();

    }

    @Override
    public boolean isFinished() {
        if (System.currentTimeMillis() < endTime) {
            return true;
        } else {
            return false;
        }

    }
} 
