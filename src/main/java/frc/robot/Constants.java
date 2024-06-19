package frc.robot;



import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import java.util.Map;
import java.util.function.BooleanSupplier;

public class Constants {

	public static boolean isComp;
	public static boolean isEpsilon;

	public static BooleanSupplier isCompSupplier() {
		return () -> isComp;
	}

	public static final String kCompSerial = "032B4B47";
	public static final String kEpsilonSerial = "03260A21";

	// Disables extra smart dashboard outputs that slow down the robot
	public static final boolean disableExtraTelemetry = false;

	// robot loop time
	public static final double kLooperDt = 0.02;

	/* Control Board */
	public static final double kJoystickThreshold = 0.2;
	public static final int kButtonGamepadPort = 1;

	public static final double stickDeadband = 0.15;

	// Timeout constants
	public static final int kLongCANTimeoutMs = 100;
	public static final int kCANTimeoutMs = 10;

	public static final class ShooterConstants{
		public static final double kCompGearRatio = 1.6;
		public static final double kEpsilonTopGearRatio = 1.6;
		public static final double kEpsilonBottomGearRatio = 1.6;
		public static final double kTopFlywheelVelocityConversion =
				(60.0) * (isEpsilon ? kEpsilonTopGearRatio : kCompGearRatio) / (1.0);
		public static final double kBottomFlywheelVelocityConversion =
				(60.0) * (isEpsilon ? kEpsilonBottomGearRatio : kCompGearRatio) / (1.0);
		public static final double kFlywheelTolerance = 1000;

		public static TalonFXConfiguration ShooterFXConfig() {
			TalonFXConfiguration config = new TalonFXConfiguration();

			config.Slot0.kP = 0.0;
			config.Slot0.kI = 0.0;
			config.Slot0.kD = 0.0;
			config.Slot0.kV = 12.0 / (100.0);
			config.Slot0.kS = 0.15;

			config.CurrentLimits.SupplyCurrentLimit = 20.0;
			config.CurrentLimits.SupplyCurrentLimitEnable = true;
			config.CurrentLimits.SupplyTimeThreshold = 0.5;

			config.CurrentLimits.StatorCurrentLimitEnable = true;
			config.CurrentLimits.StatorCurrentLimit = 80;

			config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
			return config;
		}
	}
	

	public static final class IntakeRollerConstants {
		public static TalonFXConfiguration RollerFXConfig() {
			TalonFXConfiguration config = new TalonFXConfiguration();

			config.CurrentLimits.SupplyCurrentLimitEnable = true;
			config.CurrentLimits.SupplyCurrentLimit = 40.0;

			config.CurrentLimits.StatorCurrentLimitEnable = true;
			config.CurrentLimits.StatorCurrentLimit = 80.0;

			config.Voltage.PeakForwardVoltage = 12.0;
			config.Voltage.PeakReverseVoltage = -12.0;

			config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
			return config;
		}
	}



	public static final class IntakeConstants {
		
		
        public static TalonFXConfiguration IntakeFXConfig() {
			TalonFXConfiguration config = new TalonFXConfiguration();


			config.CurrentLimits.SupplyCurrentLimit = 20.0;
			config.CurrentLimits.SupplyCurrentLimitEnable = true;
			config.CurrentLimits.SupplyTimeThreshold = 0.5;

			config.CurrentLimits.StatorCurrentLimitEnable = true;
			config.CurrentLimits.StatorCurrentLimit = 80;

			config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
			return config;
		}
	}

	
}