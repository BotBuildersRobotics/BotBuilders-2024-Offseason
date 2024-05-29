// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team10002.frc2024.subsystems;




import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team10002.frc2024.Constants;
import com.team10002.frc2024.Ports;
import com.team10002.frc2024.loops.ILooper;
import com.team10002.frc2024.loops.Loop;
import com.team10002.lib.requests.Request;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonUtil;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class IntakeSubsystem extends Subsystem {

  private TalonFX frontRoller;
  private TalonFX rearRoller;
  

  private static IntakeSubsystem mInstance;

  public static IntakeSubsystem getInstance() {
      if (mInstance == null) {
        mInstance = new IntakeSubsystem();
      }
      return mInstance;
  }

  public enum State {
		IDLE(0.0),
		INTAKING(8.0),
		EXHAUST(-6.0);

		public double roller_voltage;

		State(double roller_voltage) {
			this.roller_voltage = roller_voltage;
		}
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

  private State mState = State.IDLE;
  private final PeriodicIO periodicIO = new PeriodicIO();

  public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {}

			@Override
			public void onLoop(double timestamp) {
				periodicIO.roller_demand = mState.roller_voltage;
			}

			@Override
			public void onStop(double timestamp) {}
		});
	}


  private static class PeriodicIO implements Sendable {
		// Inputs
		private double roller_output_voltage;
		private double roller_stator_current;
		private double roller_velocity;

    private double rear_roller_output_voltage;
		private double rear_roller_stator_current;
		private double rear_roller_velocity;

		// Outputs
		private double roller_demand;

    @Override
		public void initSendable(SendableBuilder builder) {
			builder.addDoubleProperty("Demand", () -> roller_demand, null);
			builder.addDoubleProperty("VelocityRpS", () -> roller_velocity, null);
			builder.addDoubleProperty("OutputVoltage", () -> roller_output_voltage, null);
			builder.addDoubleProperty("StatorCurrent", () -> roller_stator_current, null);

      builder.addDoubleProperty("VelocityRpS", () -> rear_roller_velocity, null);
			builder.addDoubleProperty("RearOutputVoltage", () -> rear_roller_output_voltage, null);
			builder.addDoubleProperty("RearStatorCurrent", () -> rear_roller_stator_current, null);
		}
	}
/**
	 * Gets the current state of the intake rollers.
	 *
	 * @return The current state.
	 */
	public State getState() {
		return mState;
	}

	/**
	 * Sets the state of the intake rollers.
	 *
	 * @param state The state to set.
	 */
	public void setState(State state) {
		mState = state;
	}

  /**
	 * @param _wantedState Wanted state for the intake rollers.
	 * @return New request that updates the intake rollers with the wanted state. 
	 */
	public Request stateRequest(State _wantedState) {
		return new Request() {
			@Override
			public void act() {
				setState(_wantedState);
			}

			@Override
			public boolean isFinished() {
				return periodicIO.roller_demand == _wantedState.roller_voltage;
			}
		};
	}

  @Override
	public void readPeriodicInputs() {
		
    periodicIO.roller_output_voltage = frontRoller.getMotorVoltage().getValue();
		periodicIO.roller_stator_current = frontRoller.getStatorCurrent().getValue();
		periodicIO.roller_velocity = frontRoller.getVelocity().getValue();

    periodicIO.rear_roller_output_voltage = rearRoller.getMotorVoltage().getValue();
		periodicIO.rear_roller_stator_current = rearRoller.getStatorCurrent().getValue();
		periodicIO.rear_roller_velocity = rearRoller.getVelocity().getValue();

	}

	@Override
	public void writePeriodicOutputs() {

		frontRoller.setControl(new VoltageOut(periodicIO.roller_demand));
    rearRoller.setControl(new VoltageOut(periodicIO.roller_demand));
	}

	@Override
	public void stop() {
		periodicIO.roller_demand = 0.0;
	}

	@Override
	public boolean checkSystem() {
		return false;
	}

	@Override
	public void outputTelemetry() {
		SmartDashboard.putString("IntakeRollers/State", mState.toString());
		SmartDashboard.putData("IntakeRollers/IO", periodicIO);
	}


    
}
