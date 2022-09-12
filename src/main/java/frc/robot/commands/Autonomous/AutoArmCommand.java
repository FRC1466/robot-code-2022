package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import java.lang.Math;
import java.util.concurrent.TimeUnit;

public class AutoArmCommand extends CommandBase {
    private final IntakeSubsystem m_intake;
    private final DriveSubsystem m_drive;
    private double m_pos;
    private double lastPos;
    private boolean isMoving = true;
    private boolean isCommandFinished = false;
    private double timeToRoll;

    public AutoArmCommand(IntakeSubsystem intake, DriveSubsystem drive, double pos) {
        m_intake = intake;
        m_drive = drive;
        addRequirements(m_intake);
        addRequirements(m_drive);
        timeToRoll = Constants.AutoConstants.kRollTime*1000/50; //convert to commanderscheduler cycles (20ms per cycle)
        lastPos = IntakeConstants.armStartPos;
        m_pos = pos;
        System.out.println("Initialized AutoArmCommand class");

    }

    private void SpinRoller(boolean isSpinning) {
        if (isSpinning) {
            m_intake.runRoller(1);
        } else {
            m_intake.stopRoller();
        }
        
        
    }

    @Override
    public void initialize() {

        
    }

    @Override
    public void execute() {
        m_intake.runArm(m_pos, isMoving, lastPos);
        m_drive.pacifyDrive();
        System.out.println("Executing AutoArmCommand: " + m_intake.getCurrentError());
        
    
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(m_intake.getCurrentError()) < AutoConstants.kArmErrorFinish) {
            isCommandFinished = true;
            System.out.println("Finished Arm Movement");
        }
        return isCommandFinished;
    }

}