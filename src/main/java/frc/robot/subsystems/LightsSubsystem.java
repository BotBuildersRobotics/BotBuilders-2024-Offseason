package frc.robot.subsystems;

import javax.sound.sampled.Port;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj.CAN;
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

        if(state == LightState.GREEN) {
            led.setLEDs(0, 225, 0);
        }

        if(state == LightState.BLUE){
            led.setLEDs(0, 0, 225);
        }

    }

    public enum LightState {
        RED,
        GREEN,
        BLUE

    }
}
