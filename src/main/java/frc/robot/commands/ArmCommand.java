package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.IntakeConstants;
import java.lang.Math;

import javax.swing.text.Position;

public class ArmCommand extends CommandBase {
    private final IntakeSubsystem m_arm;
    private final XboxController m_controller;
    private boolean isMoving = true;
    private double lastPos;
    private boolean isPosLocked = false;
    private boolean isScreaming = false;
    private boolean toggleXButton = false;
    private int posIter = 0;
    private double lockedPosition;
    private double forward;

    public ArmCommand(IntakeSubsystem subsystem, XboxController controller) {
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

    private void updateSmartDashboard() {
        SmartDashboard.putNumber("Arm Sensor Vel", m_arm.getVelocity());
        SmartDashboard.putNumber("Arm Sensor Pos", m_arm.getPosition());
        SmartDashboard.putNumber("Arm Last Position", lastPos);
        SmartDashboard.putNumber("Arm Joystick", forward);
        SmartDashboard.putBoolean("Position Locked", isMoving);
    }

    @Override
    public void initialize() {
        lastPos = IntakeConstants.armStartPos;
    }

    @Override
    public void execute() {
        forward = m_controller.getLeftY();
        forward = Math.pow(forward, 3); // min and max are reversed??

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
        if (m_controller.getRightStickButton()) {
            if (!isPosLocked) {
                isPosLocked = !isPosLocked;
                lastPos = m_arm.getPosition();
                isPosLocked = !isPosLocked;
                System.out.println(lastPos + " toggled");
            }
        }

        if (m_controller.getXButtonPressed()) {
            isMoving = !isMoving;
            lockedPosition = m_arm.getPosition();
        }


        ScreamLimiter(); // make sure robot doesn't die when things go wrong
        if (isScreaming) {
            forward = 0;
            isPosLocked = false;
        }

        // m_arm.runArm(forward, isMoving, lastPos, lockedPosition);
        m_arm.runArmNormal(forward*IntakeConstants.armPower, isMoving);
        updateSmartDashboard();



        if (m_controller.getXButton()) {
            System.out.println("Sensor Vel:" + m_arm.getVelocity());
            System.out.println("Sensor Pos:" + m_arm.getPosition());
            System.out.println("Last Position:" + lastPos);
        }

    }

}
