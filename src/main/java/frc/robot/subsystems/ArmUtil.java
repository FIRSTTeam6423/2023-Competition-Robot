// package frc.robot.subsystems;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.CANSparkMax.IdleMode;

// import frc.robot.Constants;
// import frc.robot.util.ArmState;

// public class ArmUtil extends SubsystemBase{
//     private CANSparkMax armMotor1, armMotor2, wristMotor;
//     private ArmState armState;
//     private RelativeEncoder arm1Encoder, arm2Encoder, wristEncoder;
//     private DigitalInput armLimitSwitch, wristLimitSwitch;
//     private PIDController armPIDController, wristPIDController;
//     private ArmFeedforward armFeedForwardController, wristFeedForwardController;


//     public ArmUtil() {
//         armState = ArmState.INITIALIZING;
        
//         armMotor1 = new CANSparkMax(Constants.ARM1, MotorType.kBrushless);
//         armMotor2 = new CANSparkMax(Constants.ARM2, MotorType.kBrushless);
//         wristMotor = new CANSparkMax(Constants.WRIST,MotorType.kBrushless);
       
//         wristMotor.setIdleMode(IdleMode.kBrake);
//         armMotor1.setIdleMode(IdleMode.kBrake);
//         armMotor2.setIdleMode(IdleMode.kBrake);


//         armMotor1.setInverted(true); //up is positive for both arm and wrist

//         arm1Encoder = armMotor1.getEncoder();
//         arm2Encoder = armMotor2.getEncoder();
//         wristEncoder = wristMotor.getEncoder();

//         arm1Encoder.setPositionConversionFactor(Constants.ARM_TICKS_PER_DEG); //Now fixed. Problem was we were assuming native units were
//         arm2Encoder.setPositionConversionFactor(Constants.ARM_TICKS_PER_DEG); //ticks when its actually rotations. Constants show fix
//         wristEncoder.setPositionConversionFactor(Constants.WRIST_TICKS_PER_DEG); //Me personally, i would test moving arm on own, then add on wrist once code to make sure it doesnt hit bumper exists

//         armPIDController = new PIDController(Constants.ARM_P, Constants.ARM_I, Constants.ARM_D);
//         wristPIDController = new PIDController(Constants.WRIST_P, Constants.WRIST_I, Constants.WRIST_D);
        
//         armFeedForwardController = new ArmFeedforward(Constants.ARM_kS, Constants.ARM_kG, Constants.ARM_kV, Constants.ARM_kA);
//         wristFeedForwardController = new ArmFeedforward(Constants.WRIST_kS, Constants.WRIST_kG, Constants.WRIST_kV, Constants.WRIST_kA);
        
//         armLimitSwitch = new DigitalInput(Constants.ARM_LIMIT_SWITCH);  
//         wristLimitSwitch = new DigitalInput(Constants.WRIST_LIMIT_SWITCH);
//     }

//     public void resetEncoders() {
//         arm1Encoder.setPosition(0);
//         arm2Encoder.setPosition(0);
//         wristEncoder.setPosition(0);
//     }

//     public void setState(ArmState newState) {
//         armState = newState;
//     }

//     // everything is in degrees
//     //0 degrees is the limit switch
//     public void turnArm(double degrees) {
//         armMotor1.set(MathUtil.clamp(armPIDController.calculate(arm1Encoder.getPosition(), degrees), -0.5, 0.5));
//         armMotor2.set(MathUtil.clamp(armPIDController.calculate(arm2Encoder.getPosition(), degrees), -0.5, 0.5));
//     }

//     public void turnWrist(double degrees){
//         wristMotor.set(MathUtil.clamp(wristPIDController.calculate(wristEncoder.getPosition(), degrees), -0.5, 0.5));
//     }

//     public void rotateArm(double degrees){
//         armMotor1.set(MathUtil.clamp(armFeedForwardController.calculate(degrees - Constants.ARM_FEEDFORWARD_OFFSET, Constants.ARM_VELOCITY, Constants.ARM_ACCELERATION), 0.0, 2.0));// armPIDController.calculate(0, degrees));
//         armMotor2.set(MathUtil.clamp(armFeedForwardController.calculate(degrees - Constants.ARM_FEEDFORWARD_OFFSET, Constants.ARM_VELOCITY, Constants.ARM_ACCELERATION), 0, 2.0));// armPIDCo
//     }

//     public void zeroArm(){
//         arm1Encoder.setPosition(0);
//         arm2Encoder.setPosition(0);
//     }

//     public void zeroWrist(){
//         wristEncoder.setPosition(0);
//     }

//     public void operateArmToPosition(double dir){
//         if(Math.abs(dir)>=Constants.XBOX_STICK_DEADZONE_WIDTH){
//             if(dir>0){
//                 if(arm1Encoder.getPosition()< Constants.ARM_UPPER_LIMIT){
//                     armMotor1.set(0.1);//go up
//                     armMotor2.set(0.1);
//                 }
//                 else {
//                     armMotor1.set(0);
//                     armMotor2.set(0);
//                 }
//             } 
//             else {
//                 if(arm1Encoder.getPosition() > Constants.ARM_LOWER_LIMIT){
//                     //go down
//                     armMotor1.set(-0.1);
//                     armMotor2.set(-0.1);
//                 }
//                 else {
//                    armMotor1.set(0);
//                    armMotor2.set(0);
//                 }
//             }
//         }       
//     }

//     public void operateWristToPosition(double dir){
//         if(Math.abs(dir)>=Constants.XBOX_STICK_DEADZONE_WIDTH){
//             if(dir>0){
//                 if(wristEncoder.getPosition()< Constants.WRIST_UPPER_LIMIT){
//                     wristMotor.set(0.1);//go up
//                 }
//                 else {
//                     wristMotor.set(0);
//                 }
//             } 
//             else {
//                 if(wristEncoder.getPosition() > Constants.WRIST_LOWER_LIMIT){
//                     //go down
//                     wristMotor.set(-0.1);
//                 }
//                 else {
//                    wristMotor.set(0);
//                 }
//             }
//         }       
//     }

//     public boolean operateWirstToLimitSwitch(){
//         if(!wristLimitSwitch.get()) {
//             wristMotor.set(0);
//             return true;
//         }
//         wristMotor.set(.3);
//         return false;
//     }

//     public boolean operateArmToLimitSwitch() {
//         if(!armLimitSwitch.get()) {
//             armMotor1.set(0);
//             armMotor2.set(0);
//             return true;
//         }
//         armMotor1.set(-.1);
//         armMotor2.set(-.1);
//         return false;
//     }

//     public void stopWrist(){
//         // wristMotor.set(wristPIDController.calculate(wristEncoder.getPosition(), 0));
//         //Stop wrist does not work, makes motor go crazy. could cause motor to die so fix it
//         //Maybe we juse feedforward for just this? idk
//     }

//     public void operateArmStates() {
//         switch(armState) {
//             case HIGH_GOAL:
//                 turnArm(Constants.HIGH_GOAL_ARM);
//                 break;
//             case MIDDLE_GOAL:
//                 turnArm(Constants.MIDDLE_GOAL_ARM);
//                 break;
//             case LOW_GOAL:
//                 turnArm(Constants.LOW_GOAL_ARM);
//                 break;
//             case GROUND_PICK:
//                 turnArm(Constants.GROUND_PICK_ARM);
//                 break;
//             case HIGH_PICK:
//                 turnArm(Constants.HIGH_PICK_ARM);
//                 break;
//             case RETRACT:
//             case INITIALIZING:
//                 if(operateWirstToLimitSwitch())
//                     operateArmToLimitSwitch();
//                 break;
//         }
//     }

//     public void periodic() {
//         SmartDashboard.putNumber("arm1 encoder", arm1Encoder.getPosition());
//         SmartDashboard.putNumber("arm2 encoder", arm2Encoder.getPosition());
//         SmartDashboard.putNumber("wrist encoder", wristEncoder.getPosition());
//         SmartDashboard.putBoolean("arm limitswitch", armLimitSwitch.get());
//         SmartDashboard.putBoolean("wrist limitswitch", wristLimitSwitch.get());
//         if(!armLimitSwitch.get()){
//             arm1Encoder.setPosition(0);
//             arm2Encoder.setPosition(0);
//         }
//         if(!wristLimitSwitch.get()){
//             wristEncoder.setPosition(0);
//         }
//     }
// }
