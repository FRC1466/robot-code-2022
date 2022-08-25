package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.AutoConstants;

public class ComplexAuto extends SequentialCommandGroup {

    public ComplexAuto(DriveSubsystem m_drive, IntakeSubsystem m_intake) {
        addCommands(
            new AutoArmCommand(m_intake, AutoConstants.kArmUp),
            new AutoCommand(m_drive, AutoConstants.kTestForward, AutoConstants.kTestRotate)
        );
    }
}
