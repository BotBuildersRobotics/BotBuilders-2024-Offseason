// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.Timer;

public class IntakeSubsystem extends SubsystemBase {

  private TalonFX frontRoller;
  private final VelocityVoltage voltageVelocityFront = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);

  private TalonFX rearRoller;
  private final VelocityVoltage voltageVelocityRear = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
  
  private final DutyCycleOut frontOut = new DutyCycleOut(0);
  private final DutyCycleOut rearOut = new DutyCycleOut(0);

  public IntakeSubsystem() {


    frontRoller = new TalonFX(14, TunerConstants.kCANbusName);//Constants.FRONT_ROLLER_CAN_ID);
    frontRoller.setInverted(true);
    frontRoller.setNeutralMode(NeutralModeValue.Coast);

    rearRoller = new TalonFX(15, TunerConstants.kCANbusName);//Constants.REAR_ROLLER_CAN_ID);
    rearRoller.setInverted(false);
    rearRoller.setNeutralMode(NeutralModeValue.Coast);
   
    TalonFXConfiguration frontConfigs = new TalonFXConfiguration();
    

    TalonFXConfiguration rearConfigs = new TalonFXConfiguration();

    
    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = frontRoller.getConfigurator().apply(frontConfigs);
      if(status.isOK())
        status = rearRoller.getConfigurator().apply(rearConfigs);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

  }

  public enum WantedState {
		INTAKE,
		IDLE,
    REVERSE
		
  }

  public enum SystemState {
		INTAKE,
		IDLE,
    REVERSE
	}

  private static class PeriodicIO {
		// INPUTS
		
		double krakenFrontRollerSpeed;
		double krakenFrontRollerTemperature;
		double krakenFrontRollerSupplyCurrent;
		double krakenFrontRollerStatorCurrent;

    double krakenRearRollerSpeed;
		double krakenRearRollerTemperature;
		double krakenRearRollerSupplyCurrent;
		double krakenRearRollerStatorCurrent;

    boolean reversed;


		// OUTPUTS
		double intakeFrontRollerPercentOutput;
    double intakeRearRollerPercentOutput;
	}

	
  private WantedState wantedState = WantedState.IDLE;
  private SystemState currentState = SystemState.IDLE;

  private final PeriodicIO periodicIO = new PeriodicIO();


  public void setWantedState(WantedState state) {
        wantedState = state;
        System.out.println("Processing:" + state.toString());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    readPeriodicInputs();
    processLoop();
    writePeriodicOuputs();
    outputTelemetry();
   

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

    
	public void readPeriodicInputs() {

		periodicIO.krakenFrontRollerSpeed =  frontRoller.getVelocity().getValueAsDouble();
		periodicIO.krakenFrontRollerTemperature = frontRoller.getDeviceTemp().getValueAsDouble();
		periodicIO.krakenFrontRollerStatorCurrent = frontRoller.getStatorCurrent().getValueAsDouble();
		periodicIO.krakenFrontRollerSupplyCurrent = frontRoller.getSupplyCurrent().getValueAsDouble();

    periodicIO.krakenRearRollerSpeed = rearRoller.getVelocity().getValueAsDouble();
		periodicIO.krakenRearRollerTemperature = rearRoller.getDeviceTemp().getValueAsDouble();
		periodicIO.krakenRearRollerStatorCurrent = rearRoller.getStatorCurrent().getValueAsDouble();
		periodicIO.krakenRearRollerSupplyCurrent = rearRoller.getSupplyCurrent().getValueAsDouble();
        

	}
 
  public void outputTelemetry() {

      SmartDashboard.putString("intake/currentState", currentState.toString());
      
      SmartDashboard.putNumber("intake/outputFrontVel", periodicIO.intakeFrontRollerPercentOutput);
      SmartDashboard.putNumber("intake/outputRearVel", periodicIO.intakeRearRollerPercentOutput);
      
      SmartDashboard.putNumber("intake/supplyFrontStatorCurrent", periodicIO.krakenFrontRollerStatorCurrent);
      SmartDashboard.putNumber("intake/supplyRearStatorCurrent", periodicIO.krakenRearRollerStatorCurrent);
      
      SmartDashboard.putNumber("intake/frontSpeed", periodicIO.krakenFrontRollerSpeed);
      SmartDashboard.putNumber("intake/rearSpeed", periodicIO.krakenRearRollerSpeed);

      SmartDashboard.putNumber("intake/frontTemp", periodicIO.krakenFrontRollerTemperature);
      SmartDashboard.putNumber("intake/rearTemp", periodicIO.krakenRearRollerTemperature);

      SmartDashboard.putNumber("intake/frontSupplyCurrent", periodicIO.krakenFrontRollerSupplyCurrent);
      SmartDashboard.putNumber("intake/rearSupplyCurrent", periodicIO.krakenRearRollerSupplyCurrent);
      
  }

  public void writePeriodicOuputs()
  {
      if(currentState == SystemState.IDLE){
          frontRoller.setControl(new CoastOut());
          rearRoller.setControl(new CoastOut());
      }else{
      
          frontOut.EnableFOC = true;
          rearOut.EnableFOC = true;

          if(periodicIO.reversed){
            frontRoller.setInverted(false);
            rearRoller.setInverted(true);
          }else{
            frontRoller.setInverted(true);
            rearRoller.setInverted(false);
          }
          frontRoller.setControl(frontOut.withOutput(periodicIO.intakeFrontRollerPercentOutput));
          rearRoller.setControl(rearOut.withOutput(periodicIO.intakeRearRollerPercentOutput));
          
      }
  }
   
	public void processLoop() {
		SystemState newState;
		switch (currentState) {
      case IDLE:
				newState = handleIdle();
				break;
      case REVERSE:
        newState = handleReverse();
        break;
			default:
				newState = handleIntake();
				break;
			
		}
		if (newState != currentState) {
			currentState = newState;
			
		}
	}

  private SystemState handleIntake() {
    periodicIO.reversed = false;
		periodicIO.intakeFrontRollerPercentOutput = 0.5; 
		periodicIO.intakeRearRollerPercentOutput = 0.5;
    periodicIO.reversed = false;
    return defaultStateChange();
	}

  private SystemState handleReverse(){
    periodicIO.reversed = true;
    periodicIO.intakeFrontRollerPercentOutput = 0.5; 
		periodicIO.intakeRearRollerPercentOutput = 0.5;
    return defaultStateChange();
  }

  private SystemState handleIdle() {
		periodicIO.intakeFrontRollerPercentOutput = 0;
		periodicIO.intakeRearRollerPercentOutput = 0;
    return defaultStateChange();
	}

  private SystemState defaultStateChange() {
		switch(wantedState) {
      case REVERSE:
        return SystemState.REVERSE;
			case INTAKE:
			
				//this is for more complex state changes.
                //keep simple for now.
				return SystemState.INTAKE;
			default:
			case IDLE:
				return SystemState.IDLE;
		}
	}

    
}
