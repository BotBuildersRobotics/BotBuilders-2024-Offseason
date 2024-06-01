package com.team10002.frc2024;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team10002.frc2024.subsystems.limelight.GoalTracker;

import com.team10002.frc2024.subsystems.vision.VisionDeviceConstants;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
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

			config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
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

	
	/**
	 * For standard deviations:
	 * Lower = trust more, higher = trust less
	 */
	public static final class PoseEstimatorConstants {
		public record CameraConfig(Pose3d offset, String config) {}
		;

		public static final Matrix<N2, N1> kStateStdDevs = VecBuilder.fill(Math.pow(0.2, 1), Math.pow(0.2, 1));
		public static final Matrix<N2, N1> kLocalMeasurementStdDevs =
				VecBuilder.fill(Math.pow(0.01, 1), Math.pow(0.01, 1));

		public static final Map<String, CameraConfig> cameras = Map.ofEntries(
				Map.entry(
						"right",
						new CameraConfig(
								new Pose3d(
										Units.inchesToMeters(3.071),
										Units.inchesToMeters(-6.81),
										Units.inchesToMeters(24.604),
										new Rotation3d(
												Units.degreesToRadians(0),
												Units.degreesToRadians(-18),
												Units.degreesToRadians(27))),
								"{\"camera_matrix\":[[910.3756558875847,0,809.2765926238984],[0,909.8129438903156,644.2713243574459],[0,0,1]],\"distortion_coefficients\":[0.06236712235474046,-0.062294270427656145,0.004664145480488657,-0.0006911909097633055,-0.00762026244976393],\"tag_size\":0.163,\"camera_settings\":{\"width\":1600,\"height\":1200,\"fps\":50}}")),
				Map.entry(
						"left",
						new CameraConfig(
								new Pose3d(
										Units.inchesToMeters(3.071),
										Units.inchesToMeters(6.81),
										Units.inchesToMeters(24.604),
										new Rotation3d(
												Units.degreesToRadians(0),
												Units.degreesToRadians(-18),
												Units.degreesToRadians(-27))),
								"{\"camera_matrix\":[[910.3756558875847,0,809.2765926238984],[0,909.8129438903156,644.2713243574459],[0,0,1]],\"distortion_coefficients\":[0.06236712235474046,-0.062294270427656145,0.004664145480488657,-0.0006911909097633055,-0.00762026244976393],\"tag_size\":0.163,\"camera_settings\":{\"width\":1600,\"height\":1200,\"fps\":50}}")));
	}

	public static VisionDeviceConstants kLeftVisionDevice = new VisionDeviceConstants(); // dot 13
	public static VisionDeviceConstants kRightVisionDevice = new VisionDeviceConstants(); // dot 12

	static {
		kLeftVisionDevice.kTableName = "PolarisLeft";
		kLeftVisionDevice.kRobotToCamera = new com.team254.lib.geometry.Transform2d(
				new Translation2d(Units.inchesToMeters(3.071), Units.inchesToMeters(7.325)),
				Rotation2d.fromDegrees(-27));

		kRightVisionDevice.kTableName = "PolarisRight";
		kRightVisionDevice.kRobotToCamera = new com.team254.lib.geometry.Transform2d(
				new Translation2d(Units.inchesToMeters(3.071), Units.inchesToMeters(-7.325)),
				Rotation2d.fromDegrees(27.0));
	}

	

	public static final class LimelightConstants {

		public static final double kNoteHeight = 0.0508;
		public static final double kNoteTargetOffset = 0.2;
		public static final double kMaxNoteTrackingDistance = 6.75;
		public static final double kNoteTrackEpsilon = 1.0;

		public static final String kName = "limelight";
		public static final Translation2d kRobotToCameraTranslation = new Translation2d(0.0, 0.0);
		public static final double kCameraHeightMeters = isEpsilon ? 0.59 : 0.65;
		public static final Rotation2d kCameraPitch = Rotation2d.fromDegrees(-18.0);
		public static final Rotation2d kCameraYaw = Rotation2d.fromDegrees(0.0);

		public static final GoalTracker.Configuration kNoteTrackerConstants = new GoalTracker.Configuration();

		static {
			kNoteTrackerConstants.kMaxTrackerDistance = 0.46;
			kNoteTrackerConstants.kMaxGoalTrackAge = 0.5;
			kNoteTrackerConstants.kCameraFrameRate = 30.0;
			kNoteTrackerConstants.kStabilityWeight = 1.0;
			kNoteTrackerConstants.kAgeWeight = 0.2;
			kNoteTrackerConstants.kSwitchingWeight = 0.2;
		}
	}
}
