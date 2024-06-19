package frc.robot.subsystems;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.ColorFlowAnimation;

import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import frc.robot.subsystems.LightsSubsystem.LightState;


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
        if(state == LightState.FIRE){
             led.animate(new FireAnimation());
        }

        if(state == LightState.RAINBOW){
             led.animate(new RainbowAnimation());
        }

        if(state == LightState.COLOR_FLOW_RED){
             led.animate(new ColorFlowAnimation(255,0,0));
        }
        if(state == LightState.COLOR_FLOW_GREEN){
             led.animate(new ColorFlowAnimation(0,255,0));
        }
        if(state == LightState.COLOR_FLOW_BLUE){
             led.animate(new ColorFlowAnimation(0,0,255));
        }



    }

    public enum LightState {
        RED,
        ORANGE,
        GREEN,
        BLUE,
        FIRE,
        RAINBOW,
        COLOR_FLOW_RED,
        COLOR_FLOW_GREEN,
        COLOR_FLOW_BLUE

    }
}
