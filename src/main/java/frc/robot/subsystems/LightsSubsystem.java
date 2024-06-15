package frc.robot.subsystems;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;


public class LightsSubsystem extends SubsystemBase {
    
    private CANdle led;

    public static LightsSubsystem mInstnace;

    public static LightsSubsystem getInstance() {
        if (mInstnace == null) {
            mInstnace = new LightsSubsystem();
        }
        return mInstnace;
    }
    
    public LightsSubsystem() {
        
        //set up the CANdle
        led = new CANdle(Ports.LEDS.getDeviceNumber(), Ports.LEDS.getBus());
    }

    public void setState(LightState state){

        //set the candle color

        if(state == LightState.RED){
            led.setLEDs(225, 0, 0);
        }

        if(state == LightState.ORANGE){
            led.setLEDs(255, 165, 0);
        }

        if(state == LightState.GREEN) {
            led.setLEDs(0, 225, 0);
        }

        if(state == LightState.BLUE){
            led.setLEDs(0, 0, 225);
        }

    }

    public void setStrobeState(LightState state){

        if(state == LightState.RED){
            led.animate(new StrobeAnimation(255, 0, 0));
        }

        if(state == LightState.ORANGE){
            led.animate(new StrobeAnimation(255, 165, 0));
        }

        if(state == LightState.GREEN) {
             led.animate(new StrobeAnimation(0, 255, 0));
        }

        if(state == LightState.BLUE){
             led.animate(new StrobeAnimation(0, 0, 255));
        }

    }

    public enum LightState {
        RED,
        ORANGE,
        GREEN,
        BLUE

    }
}
