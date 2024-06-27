package frc.robot.lib;

public class RegressionMap {
    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kPivotAutoAimMap =
			new InterpolatingTreeMap<>();

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kTopRollerAutoAimMap =
			new InterpolatingTreeMap<>();

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kBottomRollerAutoAimMap =
			new InterpolatingTreeMap<>();

	static {
		for (double[] pair : ShooterUtil.kPivotManualAngle) {
			kPivotAutoAimMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
		}
	}

    static {
		for (double[] pair : ShooterUtil.kTopRollerManualVoltage) {
			kTopRollerAutoAimMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
		}
	}

    static {
		for (double[] pair : ShooterUtil.kBottomRollerManualVoltage) {
			kBottomRollerAutoAimMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
		}
	}
}
