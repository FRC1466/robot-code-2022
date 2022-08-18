package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.IntakeConstants;
import java.lang.Math;

import javax.swing.text.Position;

public class ArmCommand extends CommandBase {
    private final IntakeSubsystem m_arm;
    private final Joystick m_controller;
    private boolean isMoving = true;
    private double lastPos;
    private boolean isPosLocked = false;
    private boolean isScreaming = false;
    private int posIter = 0;

    public ArmCommand(IntakeSubsystem subsystem, Joystick controller) {
        m_arm = subsystem;
        addRequirements(m_arm);
        m_controller = controller;
    }

    private void ScreamLimiter() {
        double vel = m_arm.getVelocity();
        double pow = m_arm.getPercentage();
        if (vel < 200.0 && pow > 0.3 ) {
            System.out.println("AUUUGGGGGHGH");
            isScreaming = true;
        }

    }

    @Override
    public void initialize() {
        lastPos = IntakeConstants.armStartPos;
    }

    @Override
    public void execute() {
        double forward = m_controller.getThrottle();
        /* if (Math.abs(forward) < 0.10) {
            forward = 0;
            isMoving = false;
            if (!isPosLocked) {
                isPosLocked = !isPosLocked;
                lastPos = m_arm.getPosition();
                System.out.println(lastPos + " toggled");
            }
            
        } else {
            isMoving = true;
            if (isPosLocked) {
                isPosLocked = !isPosLocked;
            }
        } */
        if (m_controller.getRawButton(11)) {
            if (!isPosLocked) {
                isPosLocked = !isPosLocked;
                lastPos = m_arm.getPosition();
                System.out.println(lastPos + " toggled");
            }
        }


        ScreamLimiter(); // make sure robot doesn't die when things go wrong
        if (isScreaming) {
            forward = 0;
            isPosLocked = false;
        }

        m_arm.runArm(forward, isMoving, lastPos);

        if (m_controller.getTrigger()) {
            System.out.println("Sensor Vel:" + m_arm.getVelocity());
            System.out.println("Sensor Pos:" + m_arm.getPosition());
            System.out.println("Last Position:" + lastPos);
        }

    }

}
