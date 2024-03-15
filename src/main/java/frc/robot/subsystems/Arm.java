package frc.robot.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import javax.swing.text.Position;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;

/*import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;*/

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase{

    /*We need methods to run the arm to positions using a rev throughbore encoder in absolute mode.
     * It is needed to read arm position and to move the arm.
     */
    private final TalonFX left;
    private final TalonFX right;
    private final CANcoder encoder;
    private final MotorOutputConfigs m_MotorOutputConfigs = new MotorOutputConfigs();
    private final CurrentLimitsConfigs m_currentLimits = new CurrentLimitsConfigs();
    private final DutyCycleOut m_output = new DutyCycleOut(0);
    private final PositionVoltage m_voltagePosition = new PositionVoltage(0,0,false,0,0,false,false,false);
    /*private final ArmFeedforward ff =
        new ArmFeedforward(
        ArmConstants.kSVolts, ArmConstants.kGVolts,
        ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);*/
    public Arm (int leftID, int rightID) {
        
        left = new TalonFX(leftID);
        right = new TalonFX(rightID);
        TalonFXConfiguration TalonFXConfigs = new TalonFXConfiguration();
        TalonFXConfigurator LeftTalonFXConfigurator = left.getConfigurator();
        TalonFXConfigurator RightTalonFXConfigurator = right.getConfigurator();
        // sets to factory default
        LeftTalonFXConfigurator.apply(new TalonFXConfiguration());
        RightTalonFXConfigurator.apply(new TalonFXConfiguration());
        //declares new cancoder       
        encoder = new CANcoder(21);
        // creates current limit for motors
        m_currentLimits.SupplyCurrentLimit = 40;
        TalonFXConfigs.CurrentLimits = m_currentLimits;
        //set current limit for left motor
        LeftTalonFXConfigurator.apply(m_currentLimits); 
        //set current limit for right motor
        RightTalonFXConfigurator.apply(m_currentLimits); 
        //inverts one motor to create rotation
        TalonFXConfigs.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
        TalonFXConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        LeftTalonFXConfigurator.apply(TalonFXConfigs);
        //INVERTS ONE MOTOR
        left.setInverted(false);
        right.setInverted(true);        
        //sets to brake mode
        left.setNeutralMode(NeutralModeValue.Brake);
        right.setNeutralMode(NeutralModeValue.Brake);
    /* User can change the configs if they want, or leave it empty for factory-default */
        /*  invert in software, 
        encoder.setInverted; */
       // strict followers ignore the leader's invert and use their own
        right.setControl(new StrictFollower(left.getDeviceID()));
        // creates PID values for motor (hopefully)
        var slot0Configs = new Slot0Configs();
        slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
        slot0Configs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
        slot0Configs.kP = ArmConstants.kP;
        slot0Configs.kI = ArmConstants.kI;
        slot0Configs.kD = ArmConstants.kD;
        m_voltagePosition.Slot = 0;
        left.getConfigurator().apply(slot0Configs);
    }

    public void runOpenLoop(double supplier) {
        if(getPos() >= ArmConstants.kUpperLimit) {
            left.set(0);;
            System.out.println("¡TOO HIGH! ¡UPPER LIMIT!");
        }
        else if(getPos() <= ArmConstants.kLowerLimit) {
            left.set(0);;
            System.out.println("¡TOO LOW! ¡LOWER LIMIT!");
        }
        else {
            left.set(supplier);
        }
    }

   /*  public void hold(TrapezoidProfile.State setpoint) {
        double feedforward = ff.calculate(setpoint.position*2*Math.PI, setpoint.velocity);
        //figure out how to convert the below part to phoenix 6
            left.setPosition(setpoint, 1);
    } */

    public void runToPosition(double setpoint) {
        //these are included safety measures. not necessary, but useful
        // if(getPos() >= ArmConstants.kUpperLimit) {
        //     left.set(0);;
        //     System.out.println("¡TOO HIGH! ¡UPPER LIMIT!");
        // }
        // else if(getPos() <= ArmConstants.kLowerLimit) {
        //     left.set(0);;
        //     System.out.println("¡TOO LOW! ¡LOWER LIMIT! " + getPos());
        // }
        // else{
        //figure out how to convert the below part to phoenix 6
        left.setControl(m_voltagePosition.withPosition(setpoint));
    }
//}
    
    public void RunArm(double setpoint) {
        left.set(setpoint);
        }
    public double getPos() {
        return encoder.getPosition().getValueAsDouble();
    }
    @Override
public void periodic() {
    SmartDashboard.putNumber("getPos",getPos());
}
}



