package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ShooterSubsystem extends SubsystemBase {

    private static final double ACCEPTABLE_RPM_ERROR = 10.0;
    private static final double ACCEPTABLE_AMP_RPM_ERROR = 2.0;

    private static final double TOP_SHOT_RPM = 90;
    private static final double BOTTOM_SHOT_RPM = 90;

    private static final double AMP_TOP_RPM = 2;
    private static final double AMP_BOTTOM_RPM = 15;

    private double customTopVoltage = 0.0;
    private double customBottomVoltage = 0.0;
    
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
        WEAK(5,5,0,0),
        LONG_SHOT(12.0, 11.5,0,0),
        AMP(0.75,2.75,0,0),
        PASS(8.0, 8.0,0,0),
        CUSTOM(0.0,0.0, 0,0),
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

   

    @Override
    public void periodic() {

        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            currentState = ShooterSystemState.IDLE;
        }

        // write outputs
        if(currentState == ShooterSystemState.CUSTOM){
            io.setBottomMotorVoltage(customBottomVoltage);
            io.setTopMotorVoltage(customTopVoltage);
        }else{
            io.setBottomMotorVoltage(currentState.voltage_bottom);
            io.setTopMotorVoltage(currentState.voltage_top);
        }

           
    }

    public boolean atSpeakerSetpoint() {
        return MathUtil.isNear(TOP_SHOT_RPM, inputs.topVelocityRPS, ACCEPTABLE_RPM_ERROR)
                && MathUtil.isNear(BOTTOM_SHOT_RPM, inputs.bottomVelocityRPS, ACCEPTABLE_RPM_ERROR);
    }

    public boolean atAmpSetpoint(){
        return MathUtil.isNear(AMP_TOP_RPM, inputs.topVelocityRPS, ACCEPTABLE_AMP_RPM_ERROR)
                && MathUtil.isNear(AMP_BOTTOM_RPM, inputs.bottomVelocityRPS, ACCEPTABLE_RPM_ERROR);
    }

    public void setVoltages(double topVoltage, double bottomVoltage){
        customTopVoltage = topVoltage;
        customBottomVoltage = bottomVoltage;
    }
   


}
