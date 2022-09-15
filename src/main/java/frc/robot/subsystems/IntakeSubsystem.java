package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PIDConstants;
import pabeles.concurrency.IntOperatorTask.Max;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

public class IntakeSubsystem extends SubsystemBase {
    
    private WPI_TalonFX[] armMotors = new WPI_TalonFX[] {
        new WPI_TalonFX(IntakeConstants.arm2),  // armMotors[0] = roller
        new WPI_TalonFX(IntakeConstants.amr1)  // armMotors[1] = arm
      }; 

    private final Encoder m_armEncoder = new Encoder(5, 6, false

    );

    private void initializeMotors() {
        armMotors[0].setNeutralMode(NeutralMode.Brake);
        armMotors[1].setNeutralMode(NeutralMode.Brake);

        armMotors[1].setInverted(TalonFXInvertType.Clockwise);
    }

    private void initializePID() {
        for (int i=0; i<armMotors.length; i++) 
        {
            armMotors[i].configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
            IntakeConstants.kPIDLoopIdx, 
            IntakeConstants.kTimeoutMs);


            /* Config the peak and nominal outputs */
            armMotors[i].configNominalOutputForward(0, IntakeConstants.kTimeoutMs);
            armMotors[i].configNominalOutputReverse(0, IntakeConstants.kTimeoutMs);
            armMotors[i].configPeakOutputForward(PIDConstants.kIntakeGains.kPeakOutput, IntakeConstants.kTimeoutMs);
            armMotors[i].configPeakOutputReverse(-PIDConstants.kIntakeGains.kPeakOutput, IntakeConstants.kTimeoutMs);

            /* Config the Velocity closed loop gains in slot0 */
            armMotors[i].config_kF(IntakeConstants.kPIDLoopIdx, PIDConstants.kIntakeGains.kF, IntakeConstants.kTimeoutMs);
            armMotors[i].config_kP(IntakeConstants.kPIDLoopIdx, PIDConstants.kIntakeGains.kP, IntakeConstants.kTimeoutMs);
            armMotors[i].config_kI(IntakeConstants.kPIDLoopIdx, PIDConstants.kIntakeGains.kI, IntakeConstants.kTimeoutMs);
            armMotors[i].config_kD(IntakeConstants.kPIDLoopIdx, PIDConstants.kIntakeGains.kD, IntakeConstants.kTimeoutMs);
        }
    }

    private void resetEncoders() {
        m_armEncoder.reset();
        System.out.println(m_armEncoder.toString());
    }

    public IntakeSubsystem() {
        initializeMotors();
        initializePID();
        resetEncoders();
    }

    public void runArm(double fwd, boolean isMoving, double lastPos) {
        // double targetVelocity_UnitsPer100ms = Constants.IntakeConstants.armPower * fwd * 2000.0 * 2048.0 / 600.0;
        double setPoint = lastPos + fwd*2048*IntakeConstants.armPosRangeModifier;
        // setPoint = Math.max(setPoint, 0);

        if (isMoving) {
            // armMotors[1].set(fwd * Constants.IntakeConstants.armPower);
            armMotors[1].set(TalonFXControlMode.Position, setPoint);
        } else {
            armMotors[1].set(TalonFXControlMode.Position, lastPos);
        }
    }

    public double getPosition() {
        return armMotors[1].getSelectedSensorPosition();
    }

    public double getVelocity() {
        return armMotors[1].getSelectedSensorVelocity();
    }

    public double getPercentage() {
        return armMotors[1].getMotorOutputPercent();
    }
    
    public double getCurrentError() {
        return armMotors[1].getClosedLoopError();
    }

    

    public void runRoller(double fwd) {
        armMotors[0].set(fwd*IntakeConstants.rollerPower);
    }

    public void stopRoller() {
        armMotors[0].set(0.0);
    }

    
}
