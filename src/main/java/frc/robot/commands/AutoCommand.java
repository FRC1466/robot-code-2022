package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.DriveSubsystem;

import java.lang.Math;

public class AutoCommand extends CommandBase {
    
    private final DriveSubsystem m_drive;
    private double m_fwd = 0;
    private double m_rot = 0;
    private boolean isDone = false;
    private double m_initialPos = 0;

    public AutoCommand(DriveSubsystem subsystem, double fwd, double rot) {
        m_drive = subsystem;
        addRequirements(m_drive);

        m_fwd = fwd;
        m_rot = rot;

        m_initialPos = m_drive.getCurrentPos()[0];
        m_fwd =+ m_initialPos;
        System.out.println("Set initial position.");
        
    }

    

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        m_drive.setPeakOutputPID(AutoConstants.kPeakOutput);
    }

    @Override
    public void execute() {
        m_drive.arcadeDriveAutoPID(m_fwd, m_rot);
        System.out.println(m_drive.getCurrentTarget()[0] - m_drive.getCurrentPos()[0]);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        if (Math.abs(m_drive.getCurrentError()[0]) < AutoConstants.kTestForwardErrorLimit) {
            isDone = true;
        }
        return isDone;
    }
}
