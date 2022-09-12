package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
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

        rot = 0.60* rot;
        forward = Math.pow(forward , 1.5); // Scaling of inputs

        if (absForw > 0.7) {
            limitIter = 10; // start timer if power is too high
        }
        if (limitIter > 0) {
            limitIter--;
            isLimit = true; // lower timer and set bool
        } else {
            isLimit = false;
        }
        
        for(int i = 0; i<pastForward.length; i++) {
            if (pastForward[i]<0&&forward>0&&isLimit==true) {
                stopIter=15;
            } else if (pastForward[i]>0&&forward<0&&isLimit==true) { // logic for limiter, set limiter for a certain time
                stopIter=15;
            }
        }

        if (stopIter > 0) {
            stopIter--;
            forward=0; // lower limiter time and set into effect limiter
        }

        /* if (isLimit) {
            System.out.println(isLimit);
        } */

        pastForward[listIter%pastForward.length] = forward; // set lists and iter
        listIter++;

        /* System.out.println(pastForward[(listIter-1)%pastForward.length]);
        System.out.println(forward); */

        m_drive.arcadeDrive(-forward, rot);
    }



    private void m_arcadeDrivePID() {
        double pos = m_controller.getRightTriggerAxis();
        double neg = m_controller.getLeftTriggerAxis();
        double forward = pos - neg;
        double rot = m_controller.getLeftX();
        double absForw = Math.abs(forward);
        double absRot = Math.abs(rot);

        rot = Math.pow(rot, 3) * 0.8; // scale inputs
        forward = forward * DriveConstants.kDrivePercentActivePID;

        if (absRot > 0.10) { // input logic
            if (absForw > 0.50) {
                rot = 1.2*rot;
            } else {
                rot = 0.6*rot;
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
