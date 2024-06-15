package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ShooterSubsystem extends SubsystemBase {
    public static ShooterSubsystem mInstance;

	public static ShooterSubsystem getInstance() {

        //Rethink this for how advantage kit does 
		if (mInstance == null) {
			mInstance = new ShooterSubsystem(new ShooterIOPhoenix6());
		}
		return mInstance;
	}

    private ShooterIO io;
    private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    public enum SystemState {
        IDLE(0.0, 0.0,0,0),
        SHOOT(12.0, 12.0,0,0),
        AMP(8.0, 8.0,0,0),
        PASS(8.0, 8.0,0,0),
        REVERSE(-6.0, -6.0,0,0);

        public double voltage_top;
        public double voltage_bottom;
        public double rpm_top;
        public double rpm_bottom;

        SystemState(double voltage_top, double voltage_bottom, double rpmBottom, double rpmTop) {
            this.voltage_bottom = voltage_bottom;
            this.voltage_top = voltage_top;
            this.rpm_bottom = rpmBottom;
            this.rpm_top = rpmTop;
        }
    }

    private SystemState currentState = SystemState.IDLE;


    private ShooterSubsystem(ShooterIO shooterIO) {
		this.io = shooterIO;
	}


    public void setWantedState(SystemState state) {
        currentState = state;
    }

    public void setWantedState(SystemState wantedState, double topRPM, double bottomRPM) {
       
        this.currentState = wantedState;

        if(currentState == SystemState.AMP && topRPM != 0){
            this.currentState = SystemState.AMP;
            this.currentState.rpm_bottom = bottomRPM;
            this.currentState.rpm_top = topRPM;
        }
    }

    @Override
    public void periodic() {

        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            currentState = SystemState.IDLE;
        }

        // write outputs
        if(currentState == SystemState.AMP && currentState.rpm_top != 0){

            io.setRPM(currentState.rpm_top, currentState.rpm_bottom);

        }else{
            io.setBottomMotorVoltage(currentState.voltage_bottom);
            io.setTopMotorVoltage(currentState.voltage_top);
        }

    }

   


}
