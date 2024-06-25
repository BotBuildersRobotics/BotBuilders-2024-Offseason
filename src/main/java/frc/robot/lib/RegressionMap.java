package frc.robot.lib;

public class RegressionMap {
    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kHoodAutoAimMap =
			new InterpolatingTreeMap<>();

	static {
		for (double[] pair : ShooterUtil.kHoodManualAngle) {
			kHoodAutoAimMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
		}
	}
}
