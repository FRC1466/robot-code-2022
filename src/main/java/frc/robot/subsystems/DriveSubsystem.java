// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PIDConstants;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import java.lang.Math;

public class DriveSubsystem extends SubsystemBase {

  // motor array in a list for easy access (could do dict in future?)
  WPI_TalonFX[] motors = new WPI_TalonFX[] {
    new WPI_TalonFX(DriveConstants.kLeftMotor1Port),  // motors[0] = left1
    new WPI_TalonFX(DriveConstants.kLeftMotor2Port),  // motors[1] = left2
    new WPI_TalonFX(DriveConstants.kRightMotor1Port), // motors[2] = right1
    new WPI_TalonFX(DriveConstants.kRightMotor2Port)  // motors[3] = right2
  }; 






  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(
      new MotorControllerGroup(motors[0], motors[1]), 
      new MotorControllerGroup(motors[2], motors[3]));

  private void initializeMotors() { //set configs of motors
    for (int i=0; i<motors.length; i++) {
      motors[i].configFactoryDefault();
      motors[i].set(ControlMode.PercentOutput, 0);
      motors[i].setNeutralMode(NeutralMode.Brake);
      motors[i].configNeutralDeadband(0.001);
      if (i < 2)
      {
        motors[i].setInverted(TalonFXInvertType.CounterClockwise);
      } else {
        motors[i].setInverted(TalonFXInvertType.Clockwise);
      }
    }
  }
  

  private void initializePID() { //set configs of PID
    for (int i=0; i<motors.length; i++) 
    {
      motors[i].configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
      PIDConstants.kPIDLoopIdx, 
      PIDConstants.kTimeoutMs);


      /* Config the peak and nominal outputs */
      motors[i].configNominalOutputForward(0, PIDConstants.kTimeoutMs);
      motors[i].configNominalOutputReverse(0, PIDConstants.kTimeoutMs);
      motors[i].configPeakOutputForward(PIDConstants.kDriveGainsVelocity.kPeakOutput, PIDConstants.kTimeoutMs);
      motors[i].configPeakOutputReverse(-PIDConstants.kDriveGainsVelocity.kPeakOutput, PIDConstants.kTimeoutMs);

      /* Config the Velocity closed loop gains in slot0 */
      motors[i].config_kF(PIDConstants.kPIDLoopIdx, PIDConstants.kDriveGainsVelocity.kF, PIDConstants.kTimeoutMs);
      motors[i].config_kP(PIDConstants.kPIDLoopIdx, PIDConstants.kDriveGainsVelocity.kP, PIDConstants.kTimeoutMs);
      motors[i].config_kI(PIDConstants.kPIDLoopIdx, PIDConstants.kDriveGainsVelocity.kI, PIDConstants.kTimeoutMs);
      motors[i].config_kD(PIDConstants.kPIDLoopIdx, PIDConstants.kDriveGainsVelocity.kD, PIDConstants.kTimeoutMs);
    }
  }

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.

    initializeMotors();
    initializePID();
  }

  @Override
  public void periodic() {

  }



  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Drives the robot using PID velocity
   * 
   * @param targetFwd the target forward velocity
   * @param targetRot the target rotation velocity
   */
  public void arcadeDrivePID(double targetFwd, double targetRot) {
    double targetVelocity_UnitsPer100msRight = (targetFwd - targetRot) * 2000.0 * 2048.0 / 600.0 * DriveConstants.kDrivePercentDefaultPID;
    double targetVelocity_UnitsPer100msLeft = (targetFwd + targetRot) * 2000.0 * 2048.0 / 600.0 * DriveConstants.kDrivePercentDefaultPID;
    for (int i=0; i<(motors.length/2); i++) {
      motors[i].set(TalonFXControlMode.Velocity, targetVelocity_UnitsPer100msLeft);
      m_drive.feed(); // REALLY IMPORTANT FOR DIFFERENTIAL DRIVE!!!
      System.out.println(targetVelocity_UnitsPer100msLeft);
    }
    for (int i=(motors.length/2); i<motors.length; i++) {
      motors[i].set(TalonFXControlMode.Velocity, targetVelocity_UnitsPer100msRight);
      m_drive.feed();
      System.out.println(targetVelocity_UnitsPer100msRight);
    }
  }

  /**
   * Drives the robot using position PID
   * 
   * @param fwd the target forward position
   * @param rot the target rotation position
   */
  public void arcadeDriveAutoPID(double fwd, double rot) {
    double targetRight = (fwd - rot);
    double targetLeft = (fwd + rot);
    for (int i=0; i<(motors.length/2); i++) {
      motors[i].set(TalonFXControlMode.Position, targetLeft);
      m_drive.feed(); // REALLY IMPORTANT FOR DIFFERENTIAL DRIVE!!!
      System.out.println("fed");
    }
    for (int i=(motors.length/2); i<motors.length; i++) {
      motors[i].set(TalonFXControlMode.Position, targetRight);
      m_drive.feed();
      System.out.println("fed");
    }
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    for (WPI_TalonFX wpi_TalonFX : motors) {
      wpi_TalonFX.setVoltage(0);
    }
    m_drive.feed();
  }

  /**
   * Gets current position of motors in a list
   * @return current position of motors in a list
   */
  public double[] getCurrentPos() {
    double[] motorPos = {
      motors[0].getSelectedSensorPosition(),
      motors[1].getSelectedSensorPosition(),
      motors[2].getSelectedSensorPosition(),
      motors[3].getSelectedSensorPosition()
    };
    return motorPos;
  }

  /**
   * Gets current error of motors in a list
   * @return current error of motors in a list
   *
   */
  public double[] getCurrentError() {
    double[] motorPos = {
      motors[0].getClosedLoopError(),
      motors[1].getClosedLoopError(),
      motors[2].getClosedLoopError(),
      motors[3].getClosedLoopError()
    };
    return motorPos;
  }

  /**
   * Gets current target of motors in a list
   * @return current target of motors in a list
   *
   */
  public double[] getCurrentTarget() {
    double[] currentTarget = {
      motors[0].getClosedLoopTarget(),
      motors[1].getClosedLoopTarget(),
      motors[2].getClosedLoopTarget(),
      motors[3].getClosedLoopTarget()
    };
    return currentTarget;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /**
   * Feeds differential drive.
   */
  public void pacifyDrive() {
    m_drive.feed();
  }

  /**
   * Sets the maximum output of the PID in motors
   * @param peakOutput the peak output
   */
  public void setPeakOutputPID(double peakOutput) {
    for (int i = 0; i < motors.length; i++) {
      motors[i].configPeakOutputForward(peakOutput);
      motors[i].configPeakOutputReverse(-peakOutput);
    }
  }

}
