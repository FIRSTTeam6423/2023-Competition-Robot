package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;
import frc.robot.util.ArmState;

public class ArmUtil extends SubsystemBase{
    private CANSparkMax armMotor1, armMotor2, wristMotor;
    private ArmState state;
    private RelativeEncoder arm1Encoder, arm2Encoder, wristEncoder;
    private DigitalInput armLimitSwitch, wristLimitSwitch;

    public ArmUtil() {
        armMotor1 = new CANSparkMax(Constants.ARM1, MotorType.kBrushless);
        armMotor2 = new CANSparkMax(Constants.ARM2, MotorType.kBrushless);
        wristMotor = new CANSparkMax(Constants.WRIST,MotorType.kBrushless);
        arm1Encoder = armMotor1.getEncoder();
        arm2Encoder = armMotor2.getEncoder();
        wristEncoder = wristMotor.getEncoder();

        armLimitSwitch = new DigitalInput(Constants.ARM_LIMIT_SWITCH);
        wristLimitSwitch = new DigitalInput(Constants.WRIST_LIMIT_SWITCH);
    }

    public void resetEncoders() {
        arm1Encoder.setPositionConversionFactor(Constants.ARM_CONVERSION_FACTOR);
        arm2Encoder.setPositionConversionFactor(Constants.ARM_CONVERSION_FACTOR);
        wristEncoder.setPositionConversionFactor(Constants.ARM_CONVERSION_FACTOR);
        arm1Encoder.setPosition(0);
        arm2Encoder.setPosition(0);
        wristEncoder.setPosition(0);
    }

    public void setState(ArmState newState) {
        state = newState;
    }

    // everything is in degrees
    //0 degrees is relative to the top of the arm support
    //wrist degrees is relative to arm
    public void turn(CANSparkMax motor, RelativeEncoder encoder, double degrees) {
        double error = degrees - encoder.getPosition();
        if (error > Constants.DEADBAND && error < -Constants.DEADBAND) {
            error = degrees - encoder.getPosition();
            motor.set(error * Constants.TURN_P_VALUE);
        }

    }

    public void zeroPosition(){
        if(!armLimitSwitch.get()){
            armMotor1.set(-0.1);
            armMotor2.set(-0.1);
        }
        if(!wristLimitSwitch.get()){
            wristMotor.set(-0.1);
        }
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

    public void operateArmStates() {
        switch (state) {
            case HIGH_GOAL:
                turn(armMotor1, arm1Encoder, 120);
                turn(armMotor2,arm2Encoder,120);
                turn(wristMotor,wristEncoder,180);
                break;
            case MIDDLE_GOAL:
                turn(armMotor1,arm1Encoder,80);
                turn(armMotor2,arm2Encoder,80);
                turn(wristMotor,wristEncoder,130);
                break;
            case LOW_GOAL:
                turn(armMotor1,arm1Encoder,40);
                turn(armMotor2,arm2Encoder,40);
                turn(wristMotor,wristEncoder,80);
                break;
            case GROUND_PICK:
                turn(armMotor1,arm1Encoder,15);
                turn(armMotor2,arm2Encoder,15);
                turn(wristMotor,wristEncoder,50);
                break;
            case HIGH_PICK:
                turn(armMotor1,arm1Encoder,95);
                turn(armMotor2,arm2Encoder,95);
                turn(wristMotor,wristEncoder,120);
                break;
        }
    }

    public void periodic() {
        SmartDashboard.putNumber("arm1 encoder", arm1Encoder.getPosition());
        SmartDashboard.putNumber("arm2 encoder", arm2Encoder.getPosition());
        SmartDashboard.putNumber("wrist encoder", wristEncoder.getPosition());
        SmartDashboard.putBoolean("arm limitswitch", armLimitSwitch.get());
        SmartDashboard.putBoolean("wrist limitswitch", wristLimitSwitch.get());
    }
}
