package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ShooterSubsystem extends SubsystemBase {

    private static final double ACCEPTABLE_RPM_ERROR = 500.0;
    private static final double TOP_SHOT_RPM = 11000.0;
    private static final double BOTTOM_SHOT_RPM = 6000.0;
    private static final double AMP_TOP_RPM = 2050;
    private static final double AMP_BOTTOM_RPM = 2050;
    private static final double OUTTAKE_RPM = 1500;
    private double topFeedShotRPM = 5000;
    private double bottomFeedShotRPM = 5000;


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

    public enum ShooterSystemState {
        IDLE(0.0, 0.0,0,0),
        SHOOT(12.0, 12.0,0,0),
        AMP(8.0, 8.0,0,0),
        PASS(8.0, 8.0,0,0),
        REVERSE(-6.0, -6.0,0,0);

        public double voltage_top;
        public double voltage_bottom;
        public double rpm_top;
        public double rpm_bottom;

        ShooterSystemState(double voltage_top, double voltage_bottom, double rpmBottom, double rpmTop) {
            this.voltage_bottom = voltage_bottom;
            this.voltage_top = voltage_top;
            this.rpm_bottom = rpmBottom;
            this.rpm_top = rpmTop;
        }
    }

    private ShooterSystemState currentState = ShooterSystemState.IDLE;


    private ShooterSubsystem(ShooterIO shooterIO) {
		this.io = shooterIO;
	}


    public void setWantedState(ShooterSystemState state) {
        currentState = state;
    }

    public void setWantedState(ShooterSystemState wantedState, double topRPM, double bottomRPM) {
       
        this.currentState = wantedState;

        if(currentState == ShooterSystemState.AMP && topRPM != 0){
            this.currentState = ShooterSystemState.AMP;
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
            currentState = ShooterSystemState.IDLE;
        }

        // write outputs
        if(currentState == ShooterSystemState.AMP && currentState.rpm_top != 0){

            io.setRPM(currentState.rpm_top, currentState.rpm_bottom);

        }else{
            io.setBottomMotorVoltage(currentState.voltage_bottom);
            io.setTopMotorVoltage(currentState.voltage_top);
        }

    }

    public boolean atSpeakerSetpoint() {
        return MathUtil.isNear(TOP_SHOT_RPM, currentState.rpm_top, ACCEPTABLE_RPM_ERROR)
                && MathUtil.isNear(BOTTOM_SHOT_RPM, currentState.rpm_bottom, ACCEPTABLE_RPM_ERROR);
    }

    public boolean atAmpSetpoint(){
        return MathUtil.isNear(AMP_TOP_RPM, currentState.rpm_top, ACCEPTABLE_RPM_ERROR)
                && MathUtil.isNear(AMP_BOTTOM_RPM, currentState.rpm_bottom, ACCEPTABLE_RPM_ERROR);
    }

   


}
