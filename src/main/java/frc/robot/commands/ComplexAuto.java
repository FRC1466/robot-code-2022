package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.AutoConstants;
import java.util.concurrent.TimeUnit;

public class ComplexAuto extends SequentialCommandGroup {

    private final DriveSubsystem m_drive;

    private void waitInAuto(int a) {
        try {
            TimeUnit.MILLISECONDS.wait(a);
        } catch (Exception e) {
            System.out.println("Auto wait hasn't worked");
        }  
    }

    public ComplexAuto(DriveSubsystem drive, IntakeSubsystem m_intake) {
        addCommands(
            new AutoArmCommand(m_intake, AutoConstants.kArmUp),
            new RunCommand(() -> m_intake.runRoller(1), m_intake).withTimeout(2),
            new RunCommand(() -> m_intake.stopRoller(), m_intake).withTimeout(1),
            new AutoCommand(drive, AutoConstants.kTestForward, AutoConstants.kTestRotate)
        );

        m_drive = drive;
    }
    
}
