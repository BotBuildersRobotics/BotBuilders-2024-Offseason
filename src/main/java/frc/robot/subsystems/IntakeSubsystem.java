// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LightsSubsystem.LightState;



public class IntakeSubsystem extends SubsystemBase {


  public static IntakeSubsystem mInstance;

	public static IntakeSubsystem getInstance() {
		if (mInstance == null) {
			mInstance = new IntakeSubsystem(new IntakeIOPhoenix6());
		}
		return mInstance;
	}

  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public IntakeSubsystem(IntakeIO io) {

    this.io = io;

  }


  public enum SystemState {
		IDLE(0.0, 0.0, 0.0),
		INTAKE(8.0, 8.0, 5),
		REVERSE(-6.0, -6.0, -5),
    STAGED(0.0,0.0,0.0),
    FEEDING(0.0,0.0,12),
    SHUFFLE(0,0,0);

		public double roller_voltage_front;
    public double roller_voltage_rear;
    public double feeder_voltage;

		SystemState(double roller_voltage_front, double roller_voltage_rear, double feeder_voltage) {
			this.roller_voltage_front = roller_voltage_front;
      this.roller_voltage_rear = roller_voltage_rear;
      this.feeder_voltage = feeder_voltage;
		}
	}

  private SystemState currentState = SystemState.IDLE;



  public void setWantedState(SystemState state) {
        currentState = state;
        
  }

  @Override
  public void periodic() {
   
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
        currentState = SystemState.IDLE;
    }
    
    if(isBeamBreakTripped() && currentState != SystemState.FEEDING)
    {
       //if the beam break is tripped, we have a note in the intake, it needs to be sent out first.
       
        currentState = SystemState.STAGED;
        

    }

    

    if(currentState == SystemState.STAGED){
      LightsSubsystem.getInstance().setState(LightState.GREEN);
    }
    else if(currentState == SystemState.FEEDING){
      LightsSubsystem.getInstance().setState(LightState.RED);
    }
    else{
      LightsSubsystem.getInstance().setState(LightState.BLUE);
    }


    if(currentState != SystemState.SHUFFLE){
      io.setFrontMotorVoltage(currentState.roller_voltage_rear);
      io.setRearMotorVoltage(currentState.roller_voltage_rear);
      io.setFeederMotorVoltage(currentState.feeder_voltage);
    }
    

  }

  public void startIntake(){
    this.setWantedState(SystemState.INTAKE);
  }

  /** Returns true if the beam break is tripped */
  public boolean isBeamBreakTripped() {
      return inputs.beamBreakTripped;
  }

  
  public void SetFeederRotations(int rotations){
    this.setWantedState(SystemState.SHUFFLE);
    io.MoveFeederRotations(rotations);
  }

  public void RunFeederVoltage(int voltage){
    io.RunFeederVoltage(voltage);
  }

  public void RunCounterVoltage(int voltage){
    io.RunCounterVoltage(voltage);
  }

  public SystemState getCurrenState(){
    return currentState;
  }
    
}
