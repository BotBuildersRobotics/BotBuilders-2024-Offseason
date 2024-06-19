// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;




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


  public enum IntakeSystemState {
		IDLE(0.0, 0.0, 0.0),
		INTAKE(8.0, 8.0, 5),
		REVERSE(-6.0, -6.0, -5),
    STAGED(0.0,0.0,0.0),
    FEEDING(6.0,6.0,12),
    SHUFFLE(0,0,0);

		public double roller_voltage_front;
    public double roller_voltage_rear;
    public double feeder_voltage;

		IntakeSystemState(double roller_voltage_front, double roller_voltage_rear, double feeder_voltage) {
			this.roller_voltage_front = roller_voltage_front;
      this.roller_voltage_rear = roller_voltage_rear;
      this.feeder_voltage = feeder_voltage;
		}
	}

  private IntakeSystemState currentState = IntakeSystemState.IDLE;



  public void setWantedState(IntakeSystemState state) {
        currentState = state;
        
  }

  @Override
  public void periodic() {
   
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
        currentState = IntakeSystemState.IDLE;
    }
    
    if(isBeamBreakTripped() && currentState != IntakeSystemState.FEEDING)
    {
       //if the beam break is tripped, we have a note in the intake, it needs to be sent out first.
       
        currentState = IntakeSystemState.STAGED;

    }


    if(currentState != IntakeSystemState.SHUFFLE){
      io.setFrontMotorVoltage(currentState.roller_voltage_rear);
      io.setRearMotorVoltage(currentState.roller_voltage_rear);
      io.setFeederMotorVoltage(currentState.feeder_voltage);
    }
    

  }

  public void startIntake(){
    this.setWantedState(IntakeSystemState.INTAKE);
  }

  /** Returns true if the beam break is tripped */
  public boolean isBeamBreakTripped() {
      return inputs.beamBreakTripped;
  }

  public void SetFeederRotations(int rotations){
    this.setWantedState(IntakeSystemState.SHUFFLE);
    io.MoveFeederRotations(rotations);
  }

  public void RunFeederVoltage(int voltage){
    io.RunFeederVoltage(voltage);
  }

  public void RunCounterVoltage(int voltage){
    io.RunCounterVoltage(voltage);
  }
    
}
