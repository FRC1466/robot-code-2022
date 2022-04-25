package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class IntakeSubsystem extends SubsystemBase {
    private final WPI_TalonSRX rollerMotor = new WPI_TalonSRX(IntakeConstants.arm2);
    private final WPI_TalonFX armMotor = new WPI_TalonFX(IntakeConstants.amr1);

    public IntakeSubsystem() {
        rollerMotor.setNeutralMode(NeutralMode.Brake);
        armMotor.setNeutralMode(NeutralMode.Brake);

        armMotor.setInverted(TalonFXInvertType.Clockwise);
    }

    public void runRoller() {
        rollerMotor.setVoltage(IntakeConstants.rollerVolts);
    }

    public void stopRoller() {
        rollerMotor.setVoltage(0.0);
    }
}
