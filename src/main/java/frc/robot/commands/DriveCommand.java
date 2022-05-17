package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

import java.lang.Math;

public class DriveCommand extends CommandBase {
    private final DriveSubsystem m_drive;
    private final XboxController m_controller;
    private boolean m_isPID;
    private int limitIter = 0;
    private boolean isLimit = false;
    private int stopIter = 0;
    private int listIter = 0;
    private double pastForward[] = {0, 0, 0, 0, 0};
    
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

        if (absForw > 0.5) {
            limitIter = 20;
        }
        if (limitIter > 0) {
            limitIter--;
            isLimit = true;
        } else {
            isLimit = false;
        }
        
        for(int i = 0; i<pastForward.length; i++)
        if (pastForward[i]<0&&forward>0&&isLimit==true) {
            stopIter=25;
        } else if (pastForward[i]>0&&forward<0&&isLimit==true) {
            stopIter=25;
        }

        if (stopIter > 0) {
            stopIter--;
            forward=0;
        }

        if (isLimit) {
            System.out.println(isLimit);
        }

        pastForward[listIter%pastForward.length] = forward;
        listIter++;

        System.out.println(pastForward[(listIter-1)%pastForward.length]);
        System.out.println(forward);

        m_drive.arcadeDrive(-forward, rot);
    }



    private void m_arcadeDrivePID() {
        double pos = m_controller.getRightTriggerAxis();
        double neg = m_controller.getLeftTriggerAxis();
        double forward = pos - neg;
        double rot = m_controller.getLeftX();
        double absForw = Math.abs(forward);
        double absRot = Math.abs(rot);


        if (absRot > 0.10) {
            if (absForw > 0.50) {
                rot = 1.2*rot;
            } else {
                rot = 0.8*rot;
            }

        } else {
        rot = 0;
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
