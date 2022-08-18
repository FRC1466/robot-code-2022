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

  // motor array
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

  // The left-side drive encoder
  private final Encoder m_leftEncoder =
      new Encoder(
          DriveConstants.kLeftEncoderPorts[0],
          DriveConstants.kLeftEncoderPorts[1],
          DriveConstants.kLeftEncoderReversed);

  // The right-side drive encoder
  private final Encoder m_rightEncoder =
      new Encoder(
          DriveConstants.kRightEncoderPorts[0],
          DriveConstants.kRightEncoderPorts[1],
          DriveConstants.kRightEncoderReversed);

  // The gyro sensor
  private final Gyro m_gyro = new ADXRS450_Gyro();

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  private void initializeMotors() {

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
  

  private void initializePID() { //help there's so much
    for (int i=0; i<motors.length; i++) 
    {
      motors[i].configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
      PIDConstants.kPIDLoopIdx, 
      PIDConstants.kTimeoutMs);


      /* Config the peak and nominal outputs */
      motors[i].configNominalOutputForward(0, PIDConstants.kTimeoutMs);
      motors[i].configNominalOutputReverse(0, PIDConstants.kTimeoutMs);
      motors[i].configPeakOutputForward(PIDConstants.kGains_Velocit.kPeakOutput, PIDConstants.kTimeoutMs);
      motors[i].configPeakOutputReverse(-PIDConstants.kGains_Velocit.kPeakOutput, PIDConstants.kTimeoutMs);

      /* Config the Velocity closed loop gains in slot0 */
      motors[i].config_kF(PIDConstants.kPIDLoopIdx, PIDConstants.kGains_Velocit.kF, PIDConstants.kTimeoutMs);
      motors[i].config_kP(PIDConstants.kPIDLoopIdx, PIDConstants.kGains_Velocit.kP, PIDConstants.kTimeoutMs);
      motors[i].config_kI(PIDConstants.kPIDLoopIdx, PIDConstants.kGains_Velocit.kI, PIDConstants.kTimeoutMs);
      motors[i].config_kD(PIDConstants.kPIDLoopIdx, PIDConstants.kGains_Velocit.kD, PIDConstants.kTimeoutMs);
    }
  }

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.

    initializeMotors();
    initializePID();

    // Sets the distance per pulse for the encoders
    m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  } */

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  public void arcadeDrivePID(double targetFwd, double targetRot) {
    double targetVelocity_UnitsPer100msRight = (targetFwd - targetRot) * 2000.0 * 2048.0 / 600.0 * DriveConstants.kDrivePercentDefaultPID;
    double targetVelocity_UnitsPer100msLeft = (targetFwd + targetRot) * 2000.0 * 2048.0 / 600.0 * DriveConstants.kDrivePercentDefaultPID;
    for (int i=0; i<(motors.length/2); i++) {
      motors[i].set(TalonFXControlMode.Velocity, targetVelocity_UnitsPer100msLeft);
      System.out.println(targetVelocity_UnitsPer100msLeft);
    }
    for (int i=(motors.length/2); i<motors.length; i++) {
      motors[i].set(TalonFXControlMode.Velocity, targetVelocity_UnitsPer100msRight);
      System.out.println(targetVelocity_UnitsPer100msRight);
    }
  }

  public void arcadeDriveAutoPID(double fwd, double rot) {
    double targetRight = (fwd - rot);
    double targetLeft = (fwd + rot);
    for (int i=0; i<(motors.length/2); i++) {
      motors[i].set(TalonFXControlMode.Position, targetLeft);
    }
    for (int i=(motors.length/2); i<motors.length; i++) {
      motors[i].set(TalonFXControlMode.Position, targetRight);
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

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public Encoder getLeftEncoder() {
    return m_leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public Encoder getRightEncoder() {
    return m_rightEncoder;
  }

  public double[] getCurrentPos() {
    double[] motorPos = {
      motors[0].getSelectedSensorPosition(),
      motors[1].getSelectedSensorPosition(),
      motors[2].getSelectedSensorPosition(),
      motors[3].getSelectedSensorPosition()
    };
    return motorPos;
  }

  public double[] getCurrentError() {
    double[] motorPos = {
      motors[0].getClosedLoopError(),
      motors[1].getClosedLoopError(),
      motors[2].getClosedLoopError(),
      motors[3].getClosedLoopError()
    };
    return motorPos;
  }

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

  public void setPeakOutputPID(double peakOutput) {
    for (int i = 0; i < motors.length; i++) {
      motors[i].configPeakOutputForward(peakOutput);
      motors[i].configPeakOutputReverse(-peakOutput);
    }
  }



  /** Zeroes the heading of the robot.
  public void zeroHeading() {
    m_gyro.reset();
  }

  /*

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  } */

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   
  public double getTurnRate() {
    return -m_gyro.getRate();
  } */
}
