package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class CameraSubsystem  extends SubsystemBase {
    
    public static CameraSubsystem mInstance;

	public static CameraSubsystem getInstance() {
		if (mInstance == null) {
			mInstance = new CameraSubsystem(new CameraIOPhoton());
		}
		return mInstance;
	}

    private CameraIO io;
    private CameraIOInputsAutoLogged inputs = new CameraIOInputsAutoLogged();

    public CameraSubsystem(CameraIO io) {

        this.io = io;

    }

    @Override
    public void periodic(){

        io.updateInputs(inputs);
        Logger.processInputs("Camera", inputs);
       

    }

    public PoseCalcResult getPoseCalculation(){
        return io.getEstimatedPose();
    }
}
