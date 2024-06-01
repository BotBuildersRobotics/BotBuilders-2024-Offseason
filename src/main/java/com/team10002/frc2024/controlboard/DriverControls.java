package com.team10002.frc2024.controlboard;

import com.team10002.frc2024.loops.CrashTracker;
import com.team10002.frc2024.subsystems.LEDs;
import com.team10002.frc2024.subsystems.Superstructure;
import com.team10002.lib.logger.LogUtil;

public class DriverControls {

	ControlBoard mControlBoard = ControlBoard.getInstance();

    Superstructure mSuperstructure = Superstructure.getInstance();

	LEDs mLEDs = LEDs.getInstance();

	/* ONE CONTROLLER */

	public void oneControllerMode() {

        
        if(mControlBoard.driver.rightTrigger.isBeingPressed()){
            mSuperstructure.intake();
            
            
        }else if(mControlBoard.driver.leftTrigger.isBeingPressed()){
            mSuperstructure.exhaustState();
           
        }else{
            mSuperstructure.intakeIdle();
           
        }

        if(mControlBoard.driver.bButton.isBeingPressed()){
             mSuperstructure.shoot();
        }else{
            mSuperstructure.shootIdle();
        }

        if(mControlBoard.driver.leftBumper.isBeingPressed()){
            mSuperstructure.fieldRelative();
        }

        
		
	}

	/* TWO CONTROLLERS */

	public void twoControllerMode() {
		
	}

}