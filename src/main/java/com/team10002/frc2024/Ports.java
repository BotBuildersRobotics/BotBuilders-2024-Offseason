package com.team10002.frc2024;

import com.team254.lib.drivers.CanDeviceId;

public class Ports {
	/*
	 * LIST OF CHANNEL AND CAN IDS
	 * spotless:off
	 */
	/* SUBSYSTEM CAN DEVICE IDS */
	public static final CanDeviceId INTAKE_TOP_ROLLER = new CanDeviceId(15, "Default Name");
	public static final CanDeviceId INTAKE_BOTTOM_ROLLER = new CanDeviceId(14, "Default Name");

	
	public static final CanDeviceId SHOOTER_TOP = new CanDeviceId(16, "Default Name");
	public static final CanDeviceId SHOOTER_BOTTOM = new CanDeviceId(17, "Default Name");


	public static final CanDeviceId LEDS = new CanDeviceId(21, "Default Name");

	public static final int PIGEON = 13;

	// spotless:on

	
}
