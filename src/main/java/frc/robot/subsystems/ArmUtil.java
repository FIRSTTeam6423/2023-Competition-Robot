package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.util.ArmState;
import frc.robot.util.WristState;

public class ArmUtil extends SubsystemBase{
    private WristState wristState;
    private ArmState armState;

    private CANSparkMax armMotor1, armMotor2, wristMotor;

    private PIDController armPIDController, wristPIDController; //both positional pid controllers
    private ArmFeedforward armFeedForwardController, wristFeedForwardController;

    private RelativeEncoder arm1Encoder, arm2Encoder, wristEncoder;
    private DigitalInput armLimitSwitch, wristLimitSwitch;

    private TrapezoidProfile armProfile;

    private double holdAngle = 0;
    private boolean holding = false;
    private boolean foldWristOut = false;

    private double armStateTimeElapsed = 0;
    private boolean previousWristToggleButtonState = false;


    public ArmUtil() {
        armProfile = new TrapezoidProfile(new Constraints(0, 0), new State(0, 0));

        armState = ArmState.INITIALIZE;
        wristState = WristState.RETRACTED;

        armMotor1 = new CANSparkMax(Constants.ARM1, MotorType.kBrushless);
        armMotor2 = new CANSparkMax(Constants.ARM2, MotorType.kBrushless);
        wristMotor = new CANSparkMax(Constants.WRIST,MotorType.kBrushless);
       
        wristMotor.setIdleMode(IdleMode.kBrake);

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
        
        armFeedForwardController = new ArmFeedforward(Constants.ARM_kS, Constants.ARM_kG/12, Constants.ARM_kV, Constants.ARM_kA); //is the kg/12 because volts and using set vs setvolatage?
        wristFeedForwardController = new ArmFeedforward(Constants.WRIST_kS, Constants.WRIST_kG, Constants.WRIST_kV, Constants.WRIST_kA);
        
        armLimitSwitch = new DigitalInput(Constants.ARM_LIMIT_SWITCH);  
        wristLimitSwitch = new DigitalInput(Constants.WRIST_LIMIT_SWITCH);
    }

    public void resetEncoders() {
        arm1Encoder.setPosition(0);
        arm2Encoder.setPosition(0);
        wristEncoder.setPosition(0);
    }

    public void setArmState(ArmState newState) {
        if(armState == ArmState.INITIALIZE) return;
        armState = newState;
        switch(armState) {
            case INITIALIZE:
            case RETRACT:
            case CONTROL:
                break;
            case HIGH_GOAL:
                armProfile = new TrapezoidProfile( //Note: high goal arm and initial position both have to assume same angle is zero
                    Constants.ARM_PROFILE_CONSTRAINTS, 
                    new State(Constants.HIGH_GOAL_ARM, 0),
                    new State(getArmAngleRelativeToGround(),0)
                );
                break;
            case MIDDLE_GOAL: 
                armProfile = new TrapezoidProfile( //Note: middle goal arm and initial position both have to assume same angle is zero
                    Constants.ARM_PROFILE_CONSTRAINTS, 
                    new State(Constants.MIDDLE_GOAL_ARM, 0),
                    new State(getArmAngleRelativeToGround(),0)
                );
                break;
            case LOW_GOAL: 
                armProfile = new TrapezoidProfile( //Note: middle goal arm and initial position both have to assume same angle is zero
                    Constants.ARM_PROFILE_CONSTRAINTS, 
                    new State(Constants.LOW_GOAL_ARM, 0),
                    new State(getArmAngleRelativeToGround(),0)
                );
                break;
            case GROUND_PICK: // preset for a ground pickup
                armProfile = new TrapezoidProfile( //Note: middle goal arm and initial position both have to assume same angle is zero
                    Constants.ARM_PROFILE_CONSTRAINTS, 
                    new State(Constants.GROUND_PICK_ARM, 0),
                    new State(getArmAngleRelativeToGround(),0)
                );
                break;
            case HIGH_PICK: // preset for a player pickup
                armProfile = new TrapezoidProfile( //Note: middle goal arm and initial position both have to assume same angle is zero
                    Constants.ARM_PROFILE_CONSTRAINTS, 
                    new State(Constants.HIGH_PICK_ARM, 0),
                    new State(getArmAngleRelativeToGround(),0)
                );
                break;
        }
        armStateTimeElapsed = 0;
    }

    public void setWristState(WristState state) {
        if(armState != ArmState.INITIALIZE) wristState = state; 
    }

    // everything is in degrees
    //0 degrees is the limit switch

    /**
     * holdArm will hold the arm at whatever degrees is inputted - should be used when the operator lets go of the joystick
    */
    public void holdArm(double degrees) {
        SmartDashboard.putNumber("desired hold", degrees);
        SmartDashboard.putNumber("actual angle", arm1Encoder.getPosition()-60);
        armMotor1.set(
            MathUtil.clamp(
                armFeedForwardController.calculate(Math.toRadians(degrees), 0)
                +  armPIDController.calculate(getArmAngleRelativeToGround(), degrees), 
                -0.3, 
                0.3
            ) 
        );       
        armMotor2.set(
            MathUtil.clamp(
                armFeedForwardController.calculate(Math.toDegrees(degrees), 0) 
                + armPIDController.calculate(getArmAngleRelativeToGround(), degrees), 
                -0.3, 
                0.3
            ) 
        );
    }
    /**
     * setArmVelocity will set the arm's velocity - should be used for operator manual control
    */
    public void setArmVelocity(double degPerSec){ //I'm not sure if you can use the same pid for velocity and position
        armMotor1.set(
            MathUtil.clamp(armFeedForwardController.calculate(
                Math.toRadians(getArmAngleRelativeToGround()), 0)
                + armPIDController.calculate(arm1Encoder.getVelocity(), degPerSec),
                -0.5,
                0.5
            )
        );
        armMotor2.set(
            MathUtil.clamp(armFeedForwardController.calculate(
                Math.toRadians(getArmAngleRelativeToGround()), 0)
                + armPIDController.calculate(arm1Encoder.getVelocity(), degPerSec),
                -0.5, 
                0.5
            )
        );
    }

    public boolean operateArmToLimitSwitch() {
        if(!armLimitSwitch.get()) {
            armMotor1.set(0);
            armMotor2.set(0);
            return true;
        }
        armMotor1.set(-.05);
        armMotor2.set(-.05);
        return false;
    }

    /**
     * Used to attain a certain preset state. Uses motion profile with pid and feedforward kG
    */
    public void operateArmPreset() {
        //feedforward controller with only kg
        //pid controller for arm position
        wristState = WristState.PARALLEL_TO_GROUND;
        operateWrist();
        State setpoint = armProfile.calculate(armStateTimeElapsed); // should generate a nice curve for our position - we don't care about velocity because we cannot
        // work with velocity, as we are not able to find kV efficiently. PID should handle velocity nicely
        SmartDashboard.putNumber("setpoint", setpoint.position);
        armMotor1.set(
            MathUtil.clamp(armFeedForwardController.calculate(
                Math.toRadians(setpoint.position), 0)
                + armPIDController.calculate(getArmAngleRelativeToGround(), setpoint.position),
                -0.3,
                0.4
            )
        );
        armMotor2.set(
            MathUtil.clamp(armFeedForwardController.calculate(
                Math.toRadians(setpoint.position), 0)
                + armPIDController.calculate(getArmAngleRelativeToGround(), setpoint.position),
                -0.3,
                0.4
            )
        );
    }


    /**
     * degrees is angle with 0 being parallel to x axis, positive increase in raising arm

     * @return double degrees
     */
    public double getArmAngleRelativeToGround(){
        return arm1Encoder.getPosition() - 82;
    }

    public void setWristAngleRelativeToGround(double degrees) {
        //degrees is angle with 0 being parallel to x axis, positive increase in angle is counter-clockwise.
        //arm1Encoder.getPosition() + wristEncoder.getPosition() + 122
        double curWristDegsRelGround = arm1Encoder.getPosition() + wristEncoder.getPosition() + 78;
        wristMotor.set(
            MathUtil.clamp(
                wristFeedForwardController.calculate(
                    Math.toRadians(degrees),
                    0
                ) + wristPIDController.calculate(
                    curWristDegsRelGround, 
                    degrees
                ),
                -0.2,
                !wristLimitSwitch.get() ? 0 : 0.2
            )
        );
        
    }

    public void operateArm(double joystickInput) {
        previousWristToggleButtonState = RobotContainer.getOperator5Button();
        switch(armState) {
            case INITIALIZE: //In initialize, the user cannot move the arm and it is zeroing itself.
                boolean armAtLimitSwitch = operateArmToLimitSwitch();
                boolean wristAtLimitSwitch = operateWristToLimitSwitch();
                if(armAtLimitSwitch && wristAtLimitSwitch){
                    armState = ArmState.CONTROL;
                    wristState = WristState.RETRACTED;
                }
                break;
            case RETRACT:
                operateArmToLimitSwitch();
                if(Math.abs(joystickInput) < Constants.ARM_JOYSTICK_INPUT_DEADBAND) armState = ArmState.CONTROL;
                break;
            case CONTROL: //In control, the user has full control over the arm.
                double input = joystickInput;

                if (Math.abs(joystickInput) < Constants.ARM_JOYSTICK_INPUT_DEADBAND) {
                    if(!holding) {
                        input = 0;
                        holding = true;
                        holdAngle = getArmAngleRelativeToGround();
                        System.out.println(input);
                        System.out.println(joystickInput);
                    }
                    holdArm(holdAngle);
                } else {
                    holding = false;
                    if(joystickInput > 0 && getArmAngleRelativeToGround() >= 10) {
                        setArmVelocity(0);
                    } else {
                        setArmVelocity(joystickInput * Constants.ARM_VELOCITY);
                    }
                }

                SmartDashboard.putBoolean("Holding?", holding);

                //if(arm1Encoder.getPosition() <= Constants.WRIST_RETRACT_DEADBAND){
                //    wristState = WristState.RETRACTED;
                //} else if(wristState == WristState.RETRACTED) { //this way it won't stop the user from making claw parallel
                //    wristState = WristState.OUT;
                //}
                operateWrist();

                break;
            case HIGH_GOAL: //preset for high goal
            case MIDDLE_GOAL: //preset for middle goal
            case LOW_GOAL: // preset for the low goal
            case GROUND_PICK: // preset for a ground pickup
            case HIGH_PICK: // preset for a player pickup
                if(Math.abs(joystickInput)>Constants.ARM_JOYSTICK_INPUT_DEADBAND){
                    setArmState(ArmState.CONTROL);
                }else {
                    operateArmPreset();
                }
                break;
        }
    }

    public boolean operateWristToLimitSwitch(){
        if(!wristLimitSwitch.get()) {
            wristMotor.set(0);
            return true;
        }
        wristMotor.set(0.25);
        return false;
    }

    public void lowerWrist(){
        if(wristEncoder.getPosition() <= Constants.WRIST_PHYSICAL_STOP + Constants.WRIST_STOP_DEADBAND) {
            wristMotor.set(0);
        } else {
            wristMotor.set(-.2);
        }
    }

    public void operateWrist() {
        switch(wristState) {
            case RETRACTED:
                operateWristToLimitSwitch();
                break;
            case OUT:
                lowerWrist();
                break;
            case PARALLEL_TO_GROUND:
                setWristAngleRelativeToGround(-10); // "0" should be parallel to the ground
                //wristMotor.set(-RobotContainer.getOperatorSlider()/10);
                break;
            case CONTROL:
                if(Math.abs(RobotContainer.getOperatorSlider()) >= 0.2){
                    wristMotor.set(-RobotContainer.getOperatorSlider() * 0.3);
                } else {
                    wristMotor.set(0);
                }
                break;
        }
    }

    public void toggleWristState(){
        if(wristState==WristState.OUT){
            wristState=WristState.PARALLEL_TO_GROUND;
        } else if(wristState==WristState.PARALLEL_TO_GROUND){
            wristState=WristState.OUT;
        } else if(wristState == WristState.CONTROL) {
            wristState = WristState.PARALLEL_TO_GROUND;
        }
    }

    public void toggleWristControl(){
        if(wristState==WristState.OUT){
            wristState=WristState.CONTROL;
        } else if(wristState==WristState.PARALLEL_TO_GROUND){
            wristState=WristState.CONTROL;
        } else if(wristState==WristState.CONTROL){
            wristState=WristState.RETRACTED;
        }
    }

    public void toggleWristRetract(){
        if(wristState == WristState.OUT) {
            wristState = WristState.RETRACTED;
        } else if (wristState == WristState.RETRACTED) {
            wristState = WristState.OUT;
        } else if (wristState == WristState.CONTROL) {
            wristState = WristState.RETRACTED;
        }
    }

    public void periodic() {
        armStateTimeElapsed += .02; //assumes periodic rate is 20ms = .02s

        if(!armLimitSwitch.get()){
            arm1Encoder.setPosition(0);
            arm2Encoder.setPosition(0);
        }
        if(!wristLimitSwitch.get()){
            wristEncoder.setPosition(0);
        }

        SmartDashboard.putNumber("Wrist Encoder", wristEncoder.getPosition());
        SmartDashboard.putBoolean("Wrist Switch", wristLimitSwitch.get());
        SmartDashboard.putNumber("Wrist Relative Angle", arm1Encoder.getPosition() + wristEncoder.getPosition() + 99);
        SmartDashboard.putNumber("arm angle", getArmAngleRelativeToGround());
    }
}