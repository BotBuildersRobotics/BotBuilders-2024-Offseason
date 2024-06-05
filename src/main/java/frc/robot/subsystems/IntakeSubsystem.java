// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.lib.TalonFXFactory;
import frc.robot.lib.TalonUtil;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.signals.NeutralModeValue;


public class IntakeSubsystem extends SubsystemBase {

  private TalonFX frontRoller;
  private TalonFX rearRoller;


  public static IntakeSubsystem mInstance;

	public static IntakeSubsystem getInstance() {
		if (mInstance == null) {
			mInstance = new IntakeSubsystem();
		}
		return mInstance;
	}


  public IntakeSubsystem() {

    frontRoller = TalonFXFactory.createDefaultTalon(Ports.INTAKE_TOP_ROLLER);
    TalonUtil.applyAndCheckConfiguration(frontRoller, Constants.IntakeConstants.IntakeFXConfig());
    frontRoller.setInverted(true);

    rearRoller = TalonFXFactory.createDefaultTalon(Ports.INTAKE_BOTTOM_ROLLER);
    TalonUtil.applyAndCheckConfiguration(rearRoller, Constants.IntakeConstants.IntakeFXConfig());
    rearRoller.setInverted(false);
    rearRoller.setNeutralMode(NeutralModeValue.Coast);

  }


  public enum SystemState {
		IDLE(0.0, 0.0),
		INTAKE(8.0, 8.0),
		REVERSE(-6.0, -6.0);

		public double roller_voltage_front;
    public double roller_voltage_rear;

		SystemState(double roller_voltage_front, double roller_voltage_rear) {
			this.roller_voltage_front = roller_voltage_front;
      this.roller_voltage_rear = roller_voltage_rear;
		}
	}

  private SystemState currentState = SystemState.IDLE;



  public void setWantedState(SystemState state) {
        currentState = state;
        
  }

  @Override
  public void periodic() {
   
    writeOuputs();

  }

    

  public void writeOuputs()
  {
      if(currentState == SystemState.IDLE){
          frontRoller.setControl(new CoastOut());
          rearRoller.setControl(new CoastOut());
      }else{
      
          frontRoller.setControl(new VoltageOut(currentState.roller_voltage_front));
          rearRoller.setControl(new VoltageOut(currentState.roller_voltage_rear));
          
          
      }
  }
   

    
}
