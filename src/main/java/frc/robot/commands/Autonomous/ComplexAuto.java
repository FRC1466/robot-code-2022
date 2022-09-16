package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.AutoConstants;
import java.util.concurrent.TimeUnit;

public class ComplexAuto extends SequentialCommandGroup {

    public ComplexAuto(DriveSubsystem drive, IntakeSubsystem m_intake) {
        addCommands(
            new AutoArmCommand(m_intake, drive, AutoConstants.kArmUp),
            new CustomWaitMillis(drive, 2000),
            new AutoRoller(m_intake, drive, true).withTimeout(2),
            new AutoRoller(m_intake, drive, false).withTimeout(2),
            new AutoCommand(drive, AutoConstants.kTestForward, AutoConstants.kTestRotate)
        );
    }
    
}
