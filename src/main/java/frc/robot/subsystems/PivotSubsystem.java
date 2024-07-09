package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import frc.robot.lib.TalonFXFactory;
import frc.robot.subsystems.IntakeSubsystem.IntakeSystemState;

public class PivotSubsystem extends SubsystemBase {
    
    public static PivotSubsystem mInstance;

    
    private double setpoint;

    public enum PivotSystemState{
        INTAKE(42),
        AMP(40),
        SPEAKER(18),
        STOW(0),
        SUBWOOFER(33),
        THIRTYFIVE(22),
        THIRTYSIX(22),
        LONG_RANGE(19),
        LOW_PASS(20),
        CUSTOM(0);

        public int angle;
       

        PivotSystemState(int angle) {
            this.angle = angle;
            
        }
    }

    
	public static PivotSubsystem getInstance() {
		if (mInstance == null) {
			mInstance = new PivotSubsystem(new PivotIOPhoenix6());
		}
		return mInstance;
	}

    private PivotIO io;
    private PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    private PivotSystemState currentState = PivotSystemState.STOW;

    public PivotSubsystem(PivotIO io) {

        this.io = io;

    }

    public void setWantedState(PivotSystemState wantedState){
        this.currentState = wantedState;
    }

    
    @Override
    public void periodic(){

        io.updateInputs(inputs);
        Logger.processInputs("Pivot", inputs);
       // SmartDashboard.putNumber("Pivot Value", setpoint);


        if(currentState == PivotSystemState.CUSTOM){
            this.io.setPivotAngle(setpoint);
        }else{
            this.io.setPivotAngle(currentState.angle);
        }
    }

    //sets the angle of the pivot
    public void setAngle(double angle)
    {
        this.currentState = PivotSystemState.CUSTOM;
        this.setpoint = angle;
        this.io.setPivotAngle(angle);
    }

    //Returns the angle of the pivot
    public double getCurrentPosition(){
        return currentState.angle;//this.io.getCurrentPosition();
    }

    public double getPivotAngle(){
        return this.io.getCurrentPosition();
    }

   


}