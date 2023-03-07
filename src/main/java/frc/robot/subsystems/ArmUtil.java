package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.ArmState;
import frc.robot.util.ControlState;

public class ArmUtil extends SubsystemBase{
    private CANSparkMax armMotor1, armMotor2, wristMotor;
    private ArmState armState;
    private ControlState controlState;
    private RelativeEncoder arm1Encoder, arm2Encoder, wristEncoder;
    private DigitalInput armLimitSwitch, wristLimitSwitch;
    private PIDController armPIDController, wristPIDController;

    public ArmUtil() {
        armMotor1 = new CANSparkMax(Constants.ARM1, MotorType.kBrushless);
        armMotor2 = new CANSparkMax(Constants.ARM2, MotorType.kBrushless);
        wristMotor = new CANSparkMax(Constants.WRIST,MotorType.kBrushless);
       
        wristMotor.setIdleMode(IdleMode.kBrake);
        armMotor1.setIdleMode(IdleMode.kBrake);
        armMotor2.setIdleMode(IdleMode.kBrake);


        armMotor1.setInverted(true); //up is positive for both arm and wrist

        arm1Encoder = armMotor1.getEncoder();
        arm2Encoder = armMotor2.getEncoder();
        wristEncoder = wristMotor.getEncoder();

        arm1Encoder.setPositionConversionFactor(Constants.ARM_CONVERSION_FACTOR); //default unit of rotation of motor for position
        arm2Encoder.setPositionConversionFactor(Constants.ARM_CONVERSION_FACTOR);
        wristEncoder.setPositionConversionFactor(Constants.ARM_CONVERSION_FACTOR);

        armPIDController = new PIDController(Constants.ARM_P, Constants.ARM_I, Constants.ARM_D);
        wristPIDController = new PIDController(Constants.WRIST_P, Constants.WRIST_I, Constants.WRIST_D);

        armLimitSwitch = new DigitalInput(Constants.ARM_LIMIT_SWITCH);
        wristLimitSwitch = new DigitalInput(Constants.WRIST_LIMIT_SWITCH);
    }

    public void resetEncoders() {
        arm1Encoder.setPosition(0);
        arm2Encoder.setPosition(0);
        wristEncoder.setPosition(0);
    }

    public void setState(ArmState newState) {
        armState = newState;
    }

    // everything is in degrees
    //0 degrees is the limit switch
    public void turn(CANSparkMax motor, RelativeEncoder encoder, double degrees, PIDController pController) {
        motor.set(pController.calculate(encoder.getPosition(), degrees));
    }

    public void zeroArm(){
        arm1Encoder.setPosition(0);
        arm2Encoder.setPosition(0);
    }

    public void zeroWrist(){
        wristEncoder.setPosition(0);
    }

    public void operateArmToPosition(double dir){
        if(Math.abs(dir)>=Constants.XBOX_STICK_DEADZONE_WIDTH){
            if(dir>0){
                if(arm1Encoder.getPosition()< Constants.ARM_UPPER_LIMIT){
                    armMotor1.set(0.1);//go up
                    armMotor2.set(0.1);
                }
                else {
                    armMotor1.set(0);
                    armMotor2.set(0);
                }
            } 
            else {
                if(arm1Encoder.getPosition() > Constants.ARM_LOWER_LIMIT){
                    //go down
                    armMotor1.set(-0.1);
                    armMotor2.set(-0.1);
                }
                else {
                   armMotor1.set(0);
                   armMotor2.set(0);
                }
            }
        }       
    }

    public void operateWristToPosition(double dir){
        if(Math.abs(dir)>=Constants.XBOX_STICK_DEADZONE_WIDTH){
            if(dir>0){
                if(arm1Encoder.getPosition()< Constants.WRIST_UPPER_LIMIT){
                    wristMotor.set(0.1);//go up
                }
                else {
                    wristMotor.set(0);
                }
            } 
            else {
                if(arm1Encoder.getPosition() > Constants.WRIST_LOWER_LIMIT){
                    //go down
                    wristMotor.set(-0.1);
                }
                else {
                   wristMotor.set(0);
                }
            }
        }       
    }

    public boolean operateWirstToLimitSwitch(){
        if(!wristLimitSwitch.get()) {
            wristMotor.set(0);
            return true;
        }
        wristMotor.set(.3);
        return false;
    }

    public boolean operateArmToLimitSwitch() {
        if(!armLimitSwitch.get()) {
            armMotor1.set(0);
            armMotor2.set(0);
            return true;
        }
        armMotor1.set(-.1);
        armMotor2.set(-.1);
        return false;
    }

    public void operateArmStates() {
        switch(armState) {
            case HIGH_GOAL:
                turn(armMotor1, arm1Encoder, 120, armPIDController);
                turn(armMotor2, arm2Encoder, 120, armPIDController);
                turn(wristMotor, wristEncoder, 180, wristPIDController);
                break;
            case MIDDLE_GOAL:
                turn(armMotor1, arm1Encoder, 80, armPIDController);
                turn(armMotor2, arm2Encoder, 80, armPIDController);
                turn(wristMotor, wristEncoder, 130, wristPIDController);
                break;
            case LOW_GOAL:
                turn(armMotor1, arm1Encoder, 40, armPIDController);
                turn(armMotor2, arm2Encoder, 40, armPIDController);
                turn(wristMotor, wristEncoder, 80, wristPIDController);
                break;
            case GROUND_PICK:
                turn(armMotor1, arm1Encoder, 15, armPIDController);
                turn(armMotor2, arm2Encoder, 15, armPIDController);
                turn(wristMotor, wristEncoder, 50, wristPIDController);
                break;
            case HIGH_PICK:
                turn(armMotor1, arm1Encoder, 95, armPIDController);
                turn(armMotor2, arm2Encoder, 95, armPIDController);
                turn(wristMotor, wristEncoder, 120, wristPIDController);
                break;
        }
    }

    public void operateArm(){
        // switch(controlState) {
        //     case JOYSTICK_CONTROL:
        //         operateArmToPosition(RobotContainer.getOperatorLeftXboxY());
        //         operateWristToPosition(RobotContainer.getOperatorRightXboxY());
        //         break;
        //     case BUTTON_CONTROL:
        //         operateArmStates();
        //         break;
        // }
    }

    public void periodic() {
        SmartDashboard.putNumber("arm1 encoder", arm1Encoder.getPosition());
        SmartDashboard.putNumber("arm2 encoder", arm2Encoder.getPosition());
        SmartDashboard.putNumber("wrist encoder", wristEncoder.getPosition());
        SmartDashboard.putBoolean("arm limitswitch", armLimitSwitch.get());
        SmartDashboard.putBoolean("wrist limitswitch", wristLimitSwitch.get());
        if(!armLimitSwitch.get()){
            arm1Encoder.setPosition(0);
            arm2Encoder.setPosition(0);
        }
        if(!wristLimitSwitch.get()){
            wristEncoder.setPosition(0);
        }
    }
}
