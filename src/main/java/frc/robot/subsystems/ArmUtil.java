package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.ArmState;

public class ArmUtil extends SubsystemBase{
    private CANSparkMax armMotor1, armMotor2, wristMotor;
    private ArmState armState;
    private RelativeEncoder arm1Encoder, arm2Encoder, wristEncoder;
    private DigitalInput armLimitSwitch, wristLimitSwitch;
    private PIDController armPIDController, wristPIDController;
    private ArmFeedforward armFeedForwardController, wristFeedForwardController;
    private boolean oscillationGoingUp = false;
    private double holdAngle = 0;
    private boolean holding = false;
    private boolean controlWrist = false;
    public ArmUtil() {
        armState = ArmState.INITIALIZING;
        
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

        arm1Encoder.setPositionConversionFactor(Constants.ARM_CONVERSION_FACTOR); //Now fixed. Problem was we were assuming native units were
        arm2Encoder.setPositionConversionFactor(Constants.ARM_CONVERSION_FACTOR); //ticks when its actually rotations. Constants show fix
        
        arm1Encoder.setVelocityConversionFactor(Constants.ARM_CONVERSION_FACTOR/60);
        arm2Encoder.setVelocityConversionFactor(Constants.ARM_CONVERSION_FACTOR/60);
        
        wristEncoder.setPositionConversionFactor(Constants.WRIST_CONVERSION_FACTOR); //Me personally, i would test moving arm on own, then add on wrist once code to make sure it doesnt hit bumper exists

        armPIDController = new PIDController(Constants.ARM_P, Constants.ARM_I, Constants.ARM_D);
        wristPIDController = new PIDController(Constants.WRIST_P, Constants.WRIST_I, Constants.WRIST_D);
        
        armFeedForwardController = new ArmFeedforward(Constants.ARM_kS, Constants.ARM_kG/12, Constants.ARM_kV, Constants.ARM_kA);
        wristFeedForwardController = new ArmFeedforward(Constants.WRIST_kS, Constants.WRIST_kG, Constants.WRIST_kV, Constants.WRIST_kA);
        
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

    public void holdArm(double degrees) {
        armMotor1.set(
            MathUtil.clamp(
                armFeedForwardController.calculate(Math.toRadians(degrees), 0)
                +  armPIDController.calculate(arm1Encoder.getPosition() - 60, degrees), 
                0, 
                0.3
            ) 
        );       
        armMotor2.set(
            MathUtil.clamp(
                armFeedForwardController.calculate(Math.toDegrees(degrees), 0) 
                + armPIDController.calculate(arm1Encoder.getPosition() - 60, degrees), 
                0, 
                0.3
            ) 
        );
        SmartDashboard.putNumber("Degree from horizontal", degrees);
        SmartDashboard.putNumber("Hold Arm Pos", arm1Encoder.getPosition()-60);                
    }
    public void setArmVelocity(double degPerSec){
        armMotor1.set(
            MathUtil.clamp(armFeedForwardController.calculate(
                Math.toRadians(arm1Encoder.getPosition()-60), 0)
                + armPIDController.calculate(arm1Encoder.getVelocity(), degPerSec),
                -0.3,
                0.4
            )
        );
        armMotor2.set(
            MathUtil.clamp(armFeedForwardController.calculate(
                Math.toRadians(arm1Encoder.getPosition()-60), 0)
                + armPIDController.calculate(arm1Encoder.getVelocity(), degPerSec),
                -0.3, 
                0.4
            )
        );
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

    public void operateArm(double joystickInput) {
        // armMotor2.setVoltage(Constants.ARM_kG * Math.sin(Math.toRadians(arm2Encoder.getPosition() + 30)));
        if (Math.abs(joystickInput) < Constants.ARM_JOYSTICK_INPUT_DEADBAND) {
            if(!holding) {
                holding = true;
                holdAngle = arm1Encoder.getPosition()-60;
            }
            holdArm(holdAngle);
        } else {
            holding = false;
            setArmVelocity(joystickInput * Constants.ARM_VELOCITY);
        }
        SmartDashboard.putBoolean("Holding?", holding);
    }


    public void operareWrist(boolean input) {
        if (input){
            wristMotor.set(
            MathUtil.clamp(wristFeedForwardController.calculate(Math.toRadians(arm1Encoder.getPosition() + wristEncoder.getPosition() + 142.5), 0) 
            + wristPIDController.calculate(arm1Encoder.getPosition() + wristEncoder.getPosition() + 142.5, 
            0),
            -0.3, 0.3));
        }
    }

    public void periodic() {
        SmartDashboard.putNumber("arm1 encoder", arm1Encoder.getPosition());
        SmartDashboard.putNumber("arm2 encoder", arm2Encoder.getPosition());
        SmartDashboard.putNumber("wrist encoder", wristEncoder.getPosition());
        SmartDashboard.putBoolean("wrist limitswitch", wristLimitSwitch.get());
        SmartDashboard.putNumber("wrist angle", arm1Encoder.getPosition() + wristEncoder.getPosition() + 142.5);
        if(!armLimitSwitch.get()){
            arm1Encoder.setPosition(0);
            arm2Encoder.setPosition(0);
        }
        if(!wristLimitSwitch.get()){
            wristEncoder.setPosition(0);
        }
        SmartDashboard.putNumber("wrist want", RobotContainer.getOperatorSlider() * 270);
    }
}