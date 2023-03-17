package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ClawState;

public class ClawUtil extends SubsystemBase {

    //Solenoid
    private Solenoid grabber;
    private Compressor pcmCompressor;

    /**
     * I created an enum class because it is cleaner than using a boolean.
     * Boolean is easier but sometimes folks forget that true means out and false means back.
     * With an enum we are explicitely typed and the coding is more understandable.
     * 
     * Always good to set the default state and to make sure it complies with robot 
     * start position constraints.
     */
    private ClawState state = ClawState.CLAW_CLOSE;

    public ClawUtil(){
        grabber = new Solenoid(10, PneumaticsModuleType.CTREPCM, 0);
        pcmCompressor = new Compressor(10, PneumaticsModuleType.CTREPCM);
        pcmCompressor.enableDigital();
    }

    /**
     * We are doing the bulk of the work in the Util class which is 
     * the right way to do this.  The Util class should manage its own
     * state and give helper methods to commands and other classes so that they can 
     * discover the state of the subsystem.
     * 
     * Here we are simply toggling between two states.  This can still 
     * be done with more than one state but it becomes a tad more complicated to code.
     * 
     * You may want to try expressly setting the desired state at the Command
     * and work from there in cases where you have 3+ plausible states for a subsystem.
     */
    public void toggleClaw(){
        if(state != ClawState.CLAW_OPEN){
            state = ClawState.CLAW_OPEN;
        }else{
            state = ClawState.CLAW_CLOSE;
        }
    }

    public void setState(ClawState input){
        state = input;
    }

    /**
     * Simple helper class to determine the state of the subsystem.
     * In this example we are just running it out to the SmartDashboard
     * for folks to see.
     * 
     */
    public ClawState getPistonState(){
        return state;
    }

    /**
     * This is where the actual action takes place.
     * This method is called continuously from the 
     * subsystem periodic() method.  It is checking the
     * state and making sure it is so.
     * 
     * Although this repeats over and over again, when dealing
     * with pneumatics it works and it is a no harm no foul approach.
     */
    public void operateClaw(){
        // if(state == ClawState.CLAW_CLOSE){
        //     grabber.set(true);
        // }
        // if(state == ClawState.CLAW_OPEN){
        //     grabber.set(false);
        // }
    }

    @Override
    public void periodic(){
        /**
         * Here is where we are checking the desired state and making sure it is so.
         * Additionally, we are sending state to the SmartDashboard.
         */
        //operateArm();
        //SmartDashboard.putString("Claw State :: ", getPistonState().toString());
        // SmartDashboard.putBoolean("Compressor", pcmCompressor.enabled());
        // SmartDashboard.putBoolean("Pressure Switch", pcmCompressor.getPressureSwitchValue());
        // SmartDashboard.putBoolean("Solonoid", grabber.get());
    }
}

