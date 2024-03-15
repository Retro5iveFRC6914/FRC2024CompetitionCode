package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
    private CANSparkMax leftMotor;
     private CANSparkMax rightMotor;
    private RelativeEncoder encoder;
    private SparkPIDController pid;
    private DigitalInput limitSwitch;
    private boolean homed;

    public Climber(int leftID, int rightID) {

        leftMotor = new CANSparkMax(leftID, MotorType.kBrushless);
        rightMotor = new CANSparkMax(rightID, MotorType.kBrushless);
        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();

        encoder = leftMotor.getEncoder();
        pid = leftMotor.getPIDController();

        // leftMotor.restoreFactoryDefaults();
        // rightMotor.restoreFactoryDefaults();

        leftMotor.setInverted(false);
        rightMotor.setInverted(false);

        leftMotor.setSmartCurrentLimit(40);
        rightMotor.setSmartCurrentLimit(40);

        leftMotor.enableVoltageCompensation(12.6);
        rightMotor.enableVoltageCompensation(12.6);

        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);

        rightMotor.follow(leftMotor, true);
        pid.setFeedbackDevice(encoder);

        pid.setP(ClimberConstants.kP);
        pid.setI(ClimberConstants.kI);
        pid.setD(ClimberConstants.kD);
        pid.setIZone(ClimberConstants.kIz);
        pid.setFF(ClimberConstants.kFF);
        pid.setOutputRange(ClimberConstants.kMinOutput, ClimberConstants.kMaxOutput);

    }

    // public boolean isHomed() {
    //     return homed;
    // }

    // public void home() {
    //     if(!limitSwitch.get()) {
    //         motor.set(ClimberConstants.kHomeSpeed);
    //         homed = false;
    //     }
    //     else {
    //         motor.set(0);
    //         encoder.setPosition(0);
    //         homed = true;
    //     }
    // }

    public void runOpenLoop(double speed) {
        // if(!isHomed()) {
        //     motor.set(speed);
        //     System.out.println("¡NOT HOMED! ¡OVEREXTEND POSSIBLE!");
        // }
        // else if (isHomed() && speed < 0) {
        //     motor.set(0);
        // }
        // else if (isHomed()) {
        //     if(getPos() >= ClimberConstants.kUpperLimit) {
        //         motor.set(0);
        //         System.out.println("¡CLIMBER TOO HIGH! ¡OVEREXTEND! ¡OVEREXTEND!");
        //     }
        //     else {
                leftMotor.set(speed);
            }
    //     }
    // }

    public void runToPos(double setpoint) {
        // if(!isHomed()) {
        //     motor.set(0);
        //     System.out.println("¡NOT HOMED! ¡HOMING IMPORTANT!");
        // }
        // else if(setpoint >= ClimberConstants.kLowerLimit && setpoint <= ClimberConstants.kUpperLimit) {
            pid.setReference(setpoint, ControlType.kPosition);
        }
    // }

    public void hold() {
        // if(!isHomed()) {
            leftMotor.set(0);
        // }
        // else {
            //pid.setReference(getPos(), ControlType.kPosition);
        // }
    }

    public double getPos() {
        return encoder.getPosition();
    }
    
}