package com.team10002.frc2024.controlboard;


import com.team10002.frc2024.Constants;

import com.team10002.lib.Util;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ControlBoard {
	private final double kSwerveDeadband = Constants.stickDeadband;

	private static ControlBoard mInstance = null;

	public static ControlBoard getInstance() {
		if (mInstance == null) {
			mInstance = new ControlBoard();
		}

		return mInstance;
	}

	public final CustomXboxController driver;
	//public final CustomXboxController operator;

	private ControlBoard() {
		driver = new CustomXboxController(0);
		//operator = new CustomXboxController(Constants.kButtonGamepadPort);
	}

	public void update() {
		driver.update();
		//operator.update();
	}

	
}