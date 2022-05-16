package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

import java.lang.Math;

public class DriveCommand extends CommandBase {
    private final DriveSubsystem m_drive;
    private final XboxController m_controller;
    private boolean m_isPID;
    private double pastForw[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    private int limitIter = 0;
    boolean isLimit = false;
    boolean wasPositive = true;
    int stopIter = 0;
    
    public DriveCommand(DriveSubsystem subsystem, XboxController controller, boolean PID) {
        m_drive = subsystem;
        addRequirements(m_drive);
        m_controller = controller;
        m_isPID = PID;
    }

    public void togglePID() {
        m_isPID = !m_isPID;
    }

    private void m_arcadeDrive() {
        double pos = m_controller.getRightTriggerAxis();
        double neg = m_controller.getLeftTriggerAxis();
        double forward = pos - neg;
        double rot = m_controller.getLeftX();
        double absForw = Math.abs(forward);

        if (absForw > 0.7) {
            limitIter = 5;
        }
        if (limitIter > 0) {
            limitIter--;
            isLimit = true;
        } else {
            isLimit = false;
        }

        if (wasPositive==true&&forward<0&&isLimit==true) {
            stopIter=5;
        } else if (wasPositive==false&&forward>0&&isLimit==true) {
            stopIter=5;
        }

        if (stopIter > 0) {
            stopIter--;
            forward=0;
        }

        if (forward > 0) {
            wasPositive = true;
        } else {
            wasPositive = false;
        }

        if (isLimit) {
            forward = 0;
            isLimit = false;
        }

        m_drive.arcadeDrive(-forward, rot);
    }



    private void m_arcadeDrivePID() {
        double pos = m_controller.getRightTriggerAxis();
        double neg = m_controller.getLeftTriggerAxis();
        double forward = pos - neg;
        double rot = m_controller.getLeftX();
        double absForw = Math.abs(forward);
        double absRot = Math.abs(rot);
        double pastForw[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        int iter = 0;
        double avgForw;

        pastForw[iter%pastForw.length] = absForw;
        iter++;

        double sum = 0;
        for (double d : pastForw) {
            sum += d;
        }
        avgForw = sum / pastForw.length;





        if (absRot > 0.10) {
            if (absForw > 0.50) {
                rot = 2*rot;
            } else {
                rot = 0.8*rot;
            }

        } else {
        rot = 0;
        }
        if (avgForw > 0.5) {
            forward = 0.1*forward;
        }

        m_drive.arcadeDrivePID(-forward, rot);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        if (m_isPID) {
            m_arcadeDrivePID();
        }
        else {
            m_arcadeDrive();
        }
    }
}
