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
    public static final double kDrivePercentDefault = 0.35;
    public static final double kDrivePercentActive = 0.20;
    public static final double kDrivePercentActivePID = 0.8;

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
  }

  public static final class IntakeConstants {
    public static final int amr1 = 5;
    public static final int arm2 = 4;

    public static final int armEncoderPort = 5;

    public static final double armPower = 0.30;
    public static final double rollerPower = 0.50;

    public static final int kSlotIdx = 0;
    public static final int kPIDLoopIdx = 0;
    public static final int kTimeoutMs = 30;

    public static final double armStartPos = -11500.0;
    public static final double armPosRangeModifier = 9.8;

    private static final double Ku = 4;
    private static final double Tu = 0.7;
    //   0.35,0.001,0.2                                           kP: 4   	 kI    kD      kF          Iz    PeakOut
    public final static Gains kGains_Velocit  = new Gains(0.035, 0.0000001, 2.0, 0,  0,  0.2);

  }

  public static final class PIDConstants {
    public static final int kSlotIdx = 0;
    public static final int kPIDLoopIdx = 0;
    public static final int kTimeoutMs = 30;

    //                                                    kP   	 kI    kD      kF          Iz    PeakOut
    public final static Gains kGains_Velocit  = new Gains( 0.1, 0.001, 3.5, 1023.0/20660.0,  300,  0.5);

  }

}
