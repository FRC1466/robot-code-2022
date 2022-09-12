package frc.robot.commands.Autonomous;

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
        m_fwd = -m_fwd - m_initialPos;

        
    }

    private void DebugErrors() {
        System.out.println(m_drive.getCurrentTarget()[0] - m_drive.getCurrentPos()[0]);
        System.out.println("Current Target: "+ m_drive.getCurrentTarget()[0]);
        System.out.println("Current Pos: "+ m_drive.getCurrentPos()[0]);
        System.out.println("fwd: "+ m_fwd);
    }

    

    @Override
    public void initialize() {
        m_drive.setPeakOutputPID(AutoConstants.kPeakOutput);
    }

    @Override
    public void execute() {
        m_drive.arcadeDriveAutoPID(m_fwd, m_rot);
        // DebugErrors();
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(m_drive.getCurrentError()[0]) < AutoConstants.kTestForwardErrorLimit) {
            isDone = true;
            System.out.println("Drive done, error: " + m_drive.getCurrentError());
        }
        return isDone;
    }
}
