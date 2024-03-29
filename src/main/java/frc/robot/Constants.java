// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kLeftMotor1Port = 2;
    public static final int kLeftMotor2Port = 3;
    public static final int kRightMotor1Port = 0;
    public static final int kRightMotor2Port = 1;

    public static final int[] kLeftEncoderPorts = new int[] {2, 3};
    public static final int[] kRightEncoderPorts = new int[] {0, 1};
    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = true;

    public static final double kTrackwidthMeters = 0.69;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final int kEncoderCPR = 1024;
    public static final double kWheelDiameterMeters = 0.15;
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 5;

    // Drive limiters
    public static final double kDrivePercentDefault = 0.75;
    public static final double kDrivePercentActive = 0.20;
    public static final double kDrivePercentActivePID = 0.8;
    public static final double kDrivePercentDefaultPID = 1.2;

  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int IntakePort = 1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 0.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.125;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final double kPeakOutput = 0.2;

    public static final double kTestForward = -20000;
    public static final double kTestRotate = 500.0;
    public static final double kTestForwardErrorLimit = 800.0;

    public static final double kArmUp = -0.8;
    public static final double kRollTime = 1.5;
    public static final double kArmErrorFinish = 400.0;

    public static final double kPartyRot = 0.9;
  }

  public static final class IntakeConstants {
    public static final int amr1 = 5;
    public static final int arm2 = 4;

    public static final int armEncoderPort = 5;

    public static final double armPower = 0.40;
    public static final double armInitOutput = 0.8;
    public static final double rollerPower = 3.0;

    public static final int kSlotIdx = 0;
    public static final int kPIDLoopIdx = 0;
    public static final int kTimeoutMs = 30;

    public static final double armStartPos = 0.0 ; //-15500.0
    public static final double armPosRangeModifier = 13.0*2;

  }

  public static final class PIDConstants {
    public static final int kSlotIdx = 0;
    public static final int kPIDLoopIdx = 0;
    public static final int kTimeoutMs = 30;

    //                                                    kP   	 kI    kD      kF          Iz    PeakOut
    public final static Gains kDriveGainsVelocity  = new Gains(0.4, 0.0001, 4.0, 0,  0,  0.6);
    //   0.35,0.001,0.2                                           kP: 4   	 kI    kD      kF          Iz    PeakOut
    public final static Gains kIntakeGains  = new Gains(0.034, 0.00001, 0, 0,  0,  0.25);
  }

}
