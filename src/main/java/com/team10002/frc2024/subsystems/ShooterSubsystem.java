package com.team10002.frc2024.subsystems;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team10002.frc2024.Constants;
import com.team10002.frc2024.Ports;
import com.team10002.frc2024.loops.ILooper;
import com.team10002.frc2024.loops.Loop;
import com.team10002.frc2024.subsystems.vision.VisionDevice.PeriodicIO;
import com.team10002.lib.Util;
import com.team10002.lib.requests.Request;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonUtil;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSubsystem extends Subsystem {
    
    public static ShooterSubsystem mInstance;

	public static ShooterSubsystem getInstance() {
		if (mInstance == null) {
			mInstance = new ShooterSubsystem();
		}
		return mInstance;
	}

    private final TalonFX mTopFX;
	private final TalonFX mBottomFX;

    private boolean mIsOpenLoop = false;

    private PeriodicIO mPeriodicIO = new PeriodicIO();
	private VoltageOut mRequest = new VoltageOut(0);

	private ShooterSubsystem() {
		mTopFX = TalonFXFactory.createDefaultTalon(Ports.SHOOTER_TOP);
		TalonUtil.applyAndCheckConfiguration(mTopFX, Constants.ShooterConstants.ShooterFXConfig());
		mTopFX.setInverted(false);

		mBottomFX = TalonFXFactory.createDefaultTalon(Ports.SHOOTER_BOTTOM);
		TalonUtil.applyAndCheckConfiguration(mBottomFX, Constants.ShooterConstants.ShooterFXConfig());
		mBottomFX.setInverted(true);
	}

    @Override
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {}

			@Override
			public void onLoop(double timestamp) {}

			@Override
			public void onStop(double timestamp) {
				stop();
			}
		});
	}

    @Override
	public void readPeriodicInputs() {
		mPeriodicIO.top_current = mTopFX.getSupplyCurrent().getValue();
		mPeriodicIO.top_voltage = mTopFX.getMotorVoltage().getValue();
		mPeriodicIO.top_velocity =
				mTopFX.getRotorVelocity().getValue() * Constants.ShooterConstants.kTopFlywheelVelocityConversion;

		mPeriodicIO.bottom_current = mBottomFX.getSupplyCurrent().getValue();
		mPeriodicIO.bottom_velocity =
				mBottomFX.getRotorVelocity().getValue() * Constants.ShooterConstants.kBottomFlywheelVelocityConversion;
		mPeriodicIO.bottom_voltage = mBottomFX.getMotorVoltage().getValue();

		SmartDashboard.putNumber("Shooter Voltage", mPeriodicIO.top_voltage);
	}

    @Override
	public void writePeriodicOutputs() {
		if (mIsOpenLoop) {
			// set shooter to open loop to avoid hard slowdown
			mTopFX.setControl(mRequest.withOutput(mPeriodicIO.top_demand));
			mBottomFX.setControl(mRequest.withOutput(mPeriodicIO.bottom_demand));
		} else {
			mTopFX.setControl(new VelocityVoltage(
							mPeriodicIO.top_demand / Constants.ShooterConstants.kTopFlywheelVelocityConversion)
					.withEnableFOC(false));
			mBottomFX.setControl(new VelocityVoltage(
							mPeriodicIO.bottom_demand / Constants.ShooterConstants.kBottomFlywheelVelocityConversion)
					.withEnableFOC(false));
		}
	}

    @Override
	public void outputTelemetry() {
		SmartDashboard.putData("Shooter/IO", mPeriodicIO);
		SmartDashboard.putBoolean("Shooter/Spun bot up", topSpunUp());
		SmartDashboard.putBoolean("Shooter/Spun top up", bottomSpunUp());
		SmartDashboard.putBoolean("Shooter/Spun all up", spunUp());
	}

    public static class PeriodicIO implements Sendable {
		/* Inputs */
		private double top_velocity;
		private double top_voltage;
		private double top_current;

		private double bottom_velocity;
		private double bottom_voltage;
		private double bottom_current;

		/* Outputs */
		private double top_demand;
		private double bottom_demand;

		@Override
		public void initSendable(SendableBuilder builder) {
			builder.addDoubleProperty("TopDemand", () -> top_demand, null);
			builder.addDoubleProperty("TopVelocity", () -> top_velocity, null);
			builder.addDoubleProperty("TopVoltage", () -> top_voltage, null);
			builder.addDoubleProperty("TopCurrent", () -> top_current, null);

			builder.addDoubleProperty("BottomDemand", () -> bottom_demand, null);
			builder.addDoubleProperty("BottomVelocity", () -> bottom_velocity, null);
			builder.addDoubleProperty("BottomVoltage", () -> bottom_voltage, null);
			builder.addDoubleProperty("BottomCurrent", () -> bottom_current, null);
		}
	}

    /**
	 * Sets shooter control based off voltage.
	 * Max allowed voltage is 12 volts.
	 *
	 * @param demand Voltage setpoint for both Shooter rollers.
	 */
	public void setOpenLoop(double demand) {
		if (mIsOpenLoop != true) {
			mIsOpenLoop = true;
		}
		mPeriodicIO.top_demand = demand <= 12.0 ? demand : 12.0;
		mPeriodicIO.bottom_demand = demand <= 12.0 ? demand : 12.0;
	}

    public void setVelocity(double top_velocity, double bot_velocity) {
		if (mIsOpenLoop != false) {
			mIsOpenLoop = false;
		}
		mPeriodicIO.top_demand = top_velocity;
		mPeriodicIO.bottom_demand = bot_velocity;
	}

    public synchronized boolean getIsOpenLoop() {
		return mIsOpenLoop;
	}

    public synchronized boolean spunUp() {
		return topSpunUp() && bottomSpunUp();
	}

    public synchronized boolean topSpunUp() {
		if (mPeriodicIO.top_demand >= 300) {
			boolean flywheelSpunUp = Util.epsilonEquals(
					mPeriodicIO.top_demand, mPeriodicIO.top_velocity, Constants.ShooterConstants.kFlywheelTolerance);
			return flywheelSpunUp;
		}
		return false;
	}

    public synchronized boolean bottomSpunUp() {
		if (mPeriodicIO.bottom_demand >= 300) {
			boolean flywheelSpunUp = Util.epsilonEquals(
					mPeriodicIO.bottom_demand,
					mPeriodicIO.bottom_velocity,
					Constants.ShooterConstants.kFlywheelTolerance);
			return flywheelSpunUp;
		}
		return false;
	}

    public Request prepRequest(double velocity, boolean wait) {
		return new Request() {
			@Override
			public void act() {
				setVelocity(velocity, velocity);
			}

			@Override
			public boolean isFinished() {
				return spunUp() || !wait;
			}
		};
	}

    public Request openLoopRequest(double voltage, boolean wait) {
		return new Request() {
			@Override
			public void act() {
				setOpenLoop(voltage);
			}

			@Override
			public boolean isFinished() {
				return spunUp() || !wait;
			}
		};
	}

    public Request waitRequest() {
		return new Request() {
			@Override
			public void act() {}

			@Override
			public boolean isFinished() {
				return spunUp();
			}
		};
	}

    @Override
	public void stop() {
		setOpenLoop(0.0);
	}

	public boolean checkSystem() {
		return true;
	}

}
