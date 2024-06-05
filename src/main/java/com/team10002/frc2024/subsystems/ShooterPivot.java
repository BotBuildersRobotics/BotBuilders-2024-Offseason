package com.team10002.frc2024.subsystems;

import com.team10002.frc2024.Constants;
import com.team10002.frc2024.Constants.ShooterPivotConstants;
import com.team10002.frc2024.loops.ILooper;
import com.team10002.frc2024.loops.Loop;
import com.team10002.frc2024.subsystems.servo.ServoMotorSubsystem;
import com.team10002.lib.Util;
import com.team10002.lib.requests.Request;
import com.team10002.lib.util.DelayedBoolean;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterPivot extends ServoMotorSubsystem {
    public static ShooterPivot mInstance;

    public static final double kExtensionHeight = 0.42;
	public static final double kTrapAmpHeight = 0.303;
	public static final double kRetractionHeight = 0.0;

	private boolean mHoming = false;
	private boolean mNeedsToHome = false;
	private final DelayedBoolean mHomingDelay =
			new DelayedBoolean(Timer.getFPGATimestamp(), Constants.ShooterPivotConstants.kHomingTimeout);


	public static ShooterPivot getInstance() {
		if (mInstance == null) {
			mInstance = new ShooterPivot(ShooterPivotConstants.kShooterPivotServoConstants);
		}
		return mInstance;
	}

	private ShooterPivot(final ServoMotorSubsystemConstants constants) {
		super(constants);
		zeroSensors();
		enableSoftLimits(false);
	}

    @Override
	public void registerEnabledLoops(ILooper mEnabledLooper) {
		mEnabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {
				setOpenLoop(0.0);
			}

			@Override
			public void onLoop(double timestamp) {
				// Home if we're ready to home
				if (getSetpoint() == mConstants.kHomePosition && atHomingLocation() && mNeedsToHome && !mHoming) {
					setWantHome(true);
					// If we're done homing, we no longer need to home
				} else if (mControlState != ControlState.OPEN_LOOP && mHoming) {
					setWantHome(false);
				}
			}

			@Override
			public void onStop(double timestamp) {}
		});
	}

    public void setWantHome(boolean home) {
		mHoming = home;
		if (home) {
			mNeedsToHome = false;
			mHomingDelay.update(Timer.getFPGATimestamp(), false);
		}
	}

    @Override
	public void writePeriodicOutputs() {
		if (mHoming) {
			setOpenLoop(Constants.ShooterPivotConstants.kHomingOutput / mConstants.kMaxForwardOutput);
			if (mHomingDelay.update(
					Timer.getFPGATimestamp(),
					Math.abs(getVelocity()) < Constants.ShooterPivotConstants.kHomingVelocityWindow)) {
				zeroSensors();
				setSetpointMotionMagic(mConstants.kHomePosition);
				setWantHome(false);
				mNeedsToHome = false;
			}
		}

		super.writePeriodicOutputs();
	}

	@Override
	public void stop() {
		super.stop();
	}

	@Override
	public boolean checkSystem() {
		return false;
	}

	@Override
	public boolean atHomingLocation() {
		return mPeriodicIO.position_units - mConstants.kHomePosition < Constants.ShooterPivotConstants.kHomingZone;
	}

	@Override
	public void outputTelemetry() {
		SmartDashboard.putBoolean(mConstants.kName + "/Homing", mHoming);
		SmartDashboard.putBoolean(mConstants.kName + "/Within Homing Window", atHomingLocation());

		super.outputTelemetry();
	}

    public Request angleRequest(int angle) {

        //TODO: convert angle to some number used by the motors.
        
		return new Request() {

			@Override
			public void act() {
				setSetpointMotionMagic(kRetractionHeight);
			}

			@Override
			public boolean isFinished() {
				return Util.epsilonEquals(getPosition(), kRetractionHeight, 0.1);
			}
		};
	}


}
