package com.team10002.frc2024.controlboard;

import com.team10002.frc2024.subsystems.LEDs;
import com.team10002.frc2024.subsystems.Superstructure;

public class DriverControls {

	ControlBoard mControlBoard = ControlBoard.getInstance();

    Superstructure mSuperstructure = Superstructure.getInstance();

	LEDs mLEDs = LEDs.getInstance();

	/* ONE CONTROLLER */

	public void oneControllerMode() {


        if(mControlBoard.driver.rightTrigger.wasActivated()){
            mSuperstructure.intake();
        }else if(mControlBoard.driver.leftTrigger.wasActivated()){
            mSuperstructure.exhaustState();
        }else{
            mSuperstructure.intakeIdle();
        }
		
	}

	/* TWO CONTROLLERS */

	public void twoControllerMode() {
		
	}

}