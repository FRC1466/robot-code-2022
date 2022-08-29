package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.IntakeSubsystem;

import java.lang.Math;
import java.util.concurrent.TimeUnit;

public class AutoArmCommand extends CommandBase {
    private final IntakeSubsystem m_intake;
    private double m_pos;
    private double lastPos;
    private boolean isMoving = true;
    private boolean isCommandFinished = false;
    private double timeToRoll;

    public AutoArmCommand(IntakeSubsystem intake, double pos) {
        m_intake = intake;
        m_pos = 1000.0;
        timeToRoll = Constants.AutoConstants.kRollTime*1000/50; //convert to commanderscheduler cycles (20ms per cycle)
        lastPos = IntakeConstants.armStartPos;
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
        System.out.println("Executing AutoArmCommand: " + m_intake.getCurrentError());
        
    
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(m_intake.getCurrentError()) < AutoConstants.kArmErrorFinish) {
            isCommandFinished = true;
        }
        return isCommandFinished;
    }

}