package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ComplexAuto extends SequentialCommandGroup {

    public ComplexAuto(DriveSubsystem drive, IntakeSubsystem m_intake) {
        addCommands(
            new AutoRoller(m_intake, drive, true).withTimeout(2),
            new AutoRoller(m_intake, drive, false).withTimeout(2),
            // new AutoCommand(drive, AutoConstants.kTestForward, 0)
            new RunCommand(() -> drive.arcadeDrive(0.6, 0), drive).withTimeout(1.3), // arcade drive is reversed?? TODO: unreverse it
            new RunCommand(() -> drive.arcadeDrive(0, 1), drive).withTimeout(0.6)
        );
    }
    
}
