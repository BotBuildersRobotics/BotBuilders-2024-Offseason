package frc.robot;

import frc.robot.lib.CanDeviceId;

public class Ports {
	/*
	 * LIST OF CHANNEL AND CAN IDS
	 * spotless:off
	 */
	/* SUBSYSTEM CAN DEVICE IDS */
	public static final CanDeviceId INTAKE_TOP_ROLLER = new CanDeviceId(15, "Default Name");
	public static final CanDeviceId INTAKE_BOTTOM_ROLLER = new CanDeviceId(14, "Default Name");

    public static final CanDeviceId INTAKE_FEEDER_ROLLER = new CanDeviceId(20, "Default Name");

	
	public static final CanDeviceId SHOOTER_TOP = new CanDeviceId(16, "Default Name");
	public static final CanDeviceId SHOOTER_BOTTOM = new CanDeviceId(17, "Default Name");


    public static final CanDeviceId LEADER_PIVOT = new CanDeviceId(19, "Default Name");
    public static final CanDeviceId FOLLOWER_PIVOT = new CanDeviceId(18, "Default Name");


	
	public static final CanDeviceId PIVOT_MAIN = new CanDeviceId(18, "Default Name");
	public static final CanDeviceId PIVOT_FOLLOWER = new CanDeviceId(19, "Default Name");


	public static final CanDeviceId LEDS = new CanDeviceId(21, "Default Name");

	public static final int PIGEON = 13;

	// spotless:on

	
}