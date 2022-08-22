// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.AutoCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_robotIntake = new IntakeSubsystem();
  

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  // The intake controller
  XboxController intakeController = new XboxController(OIConstants.IntakePort);

  DriveCommand m_DriveCommand = new DriveCommand(m_robotDrive, m_driverController, false);
  DriveCommand m_DriveCommandPID = new DriveCommand(m_robotDrive, m_driverController, true);

  private AutoCommand m_auto;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_robotDrive.setMaxOutput(DriveConstants.kDrivePercentDefault);
    m_auto = new AutoCommand(m_robotDrive, AutoConstants.kTestForward, AutoConstants.kTestRotate);
    System.out.println("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA: " + AutoConstants.kTestForward);
    

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        m_DriveCommand
    );

    m_robotIntake.setDefaultCommand(
        new ArmCommand(m_robotIntake, m_driverController));
  }


  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Drive at half speed when the right bumper is held
    new JoystickButton(m_driverController, Button.kB.value)
        .whenPressed(() -> m_robotDrive.setMaxOutput(DriveConstants.kDrivePercentActive))
        .whenReleased(() -> m_robotDrive.setMaxOutput(DriveConstants.kDrivePercentDefault));
    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .whenPressed(() -> m_robotIntake.runRoller(Constants.IntakeConstants.rollerPower))
        .whenReleased(() -> m_robotIntake.stopRoller());
        // Runs roller when right bumper is pressed on driver controller
    new JoystickButton(m_driverController, Button.kLeftBumper.value)
        .whenPressed(() -> m_robotIntake.runRoller(-Constants.IntakeConstants.rollerPower))
        .whenReleased(() -> m_robotIntake.stopRoller());
        // Runs roller in reverse when left bumper is pressed on driver controller
    new JoystickButton(m_driverController, Button.kA.value)
        .whenPressed(m_DriveCommandPID)
        .whenReleased(m_DriveCommand);
    }
    



  /**
   * Use this to pass the  autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  private void AutoWait(int millis) {
    try {
        Thread.sleep(millis);
    } catch (Exception e){}
  }

  public Command getAutonomousCommand() {
    return m_auto;
  }


}
