package frc.robot.subsystems;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.lib.TalonFXFactory;
import frc.robot.lib.TalonUtil;

public class ShooterSubsystem extends SubsystemBase {
    public static ShooterSubsystem mInstance;

	public static ShooterSubsystem getInstance() {
		if (mInstance == null) {
			mInstance = new ShooterSubsystem();
		}
		return mInstance;
	}


  public enum SystemState {
    IDLE(0.0, 0.0),
    SHOOT(8.0, 8.0),
    AMP(8.0, 8.0),
    PASS(8.0, 8.0),
    REVERSE(-6.0, -6.0);

    public double voltage_top;
    public double voltage_bottom;

    SystemState(double voltage_top, double voltage_bottom) {
        this.voltage_bottom = voltage_bottom;
        this.voltage_top = voltage_top;
    }
}

    private SystemState currentState = SystemState.IDLE;


    private final TalonFX mTopFX;
	private final TalonFX mBottomFX;

    private VoltageOut mRequest = new VoltageOut(0);


    private ShooterSubsystem() {
		mTopFX = TalonFXFactory.createDefaultTalon(Ports.SHOOTER_TOP);
		TalonUtil.applyAndCheckConfiguration(mTopFX, Constants.ShooterConstants.ShooterFXConfig());
		mTopFX.setInverted(false);

		mBottomFX = TalonFXFactory.createDefaultTalon(Ports.SHOOTER_BOTTOM);
		TalonUtil.applyAndCheckConfiguration(mBottomFX, Constants.ShooterConstants.ShooterFXConfig());
		mBottomFX.setInverted(true);
	}


    public void setWantedState(SystemState state) {
        currentState = state;
    }

    @Override
    public void periodic() {
   
            writeOuputs();

    }

    public void writeOuputs(){
        mTopFX.setControl(mRequest.withOutput(currentState.voltage_top));
		mBottomFX.setControl(mRequest.withOutput(currentState.voltage_bottom));
    }


}
