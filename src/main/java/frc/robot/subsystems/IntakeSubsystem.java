package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PIDConstants;
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
            PIDConstants.kPIDLoopIdx, 
            PIDConstants.kTimeoutMs);


            /* Config the peak and nominal outputs */
            armMotors[i].configNominalOutputForward(0, PIDConstants.kTimeoutMs);
            armMotors[i].configNominalOutputReverse(0, PIDConstants.kTimeoutMs);
            armMotors[i].configPeakOutputForward(PIDConstants.kGains_Velocit.kPeakOutput, PIDConstants.kTimeoutMs);
            armMotors[i].configPeakOutputReverse(-PIDConstants.kGains_Velocit.kPeakOutput, PIDConstants.kTimeoutMs);

            /* Config the Velocity closed loop gains in slot0 */
            armMotors[i].config_kF(PIDConstants.kPIDLoopIdx, PIDConstants.kGains_Velocit.kF, PIDConstants.kTimeoutMs);
            armMotors[i].config_kP(PIDConstants.kPIDLoopIdx, PIDConstants.kGains_Velocit.kP, PIDConstants.kTimeoutMs);
            armMotors[i].config_kI(PIDConstants.kPIDLoopIdx, PIDConstants.kGains_Velocit.kI, PIDConstants.kTimeoutMs);
            armMotors[i].config_kD(PIDConstants.kPIDLoopIdx, PIDConstants.kGains_Velocit.kD, PIDConstants.kTimeoutMs);
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

    public void runArm(double fwd, boolean isMoving) {
        double targetVelocity_UnitsPer100ms = Constants.IntakeConstants.armPower * fwd * 2000.0 * 2048.0 / 600.0;
        if (isMoving) {
            armMotors[1].set(TalonFXControlMode.Velocity, targetVelocity_UnitsPer100ms);
        }
            armMotors[1].set(fwd*Constants.IntakeConstants.armPower);
    }

    public void runRoller(double fwd) {
        armMotors[0].set(fwd);
    }

    public void stopRoller() {
        armMotors[0].set(0.0);
    }

    
}
