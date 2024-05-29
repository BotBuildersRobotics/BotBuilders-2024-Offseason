// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team10002.frc2024;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.team10002.frc2024.controlboard.ControlBoard;
import com.team10002.frc2024.controlboard.DriverControls;
import com.team10002.frc2024.generated.TunerConstants;
import com.team10002.frc2024.loops.CrashTracker;
import com.team10002.frc2024.loops.Looper;
import com.team10002.frc2024.subsystems.CommandSwerveDrivetrain;
import com.team10002.frc2024.subsystems.IntakeSubsystem;
import com.team10002.frc2024.subsystems.limelight.Limelight;
import com.team10002.frc2024.subsystems.limelight.Limelight.Pipeline;
import com.team10002.frc2024.subsystems.vision.VisionDeviceManager;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  //private RobotContainer m_robotContainer;

  private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();
  private final ControlBoard mControlBoard = ControlBoard.getInstance();
	private final DriverControls mDriverControls = new DriverControls();
	

// vision
	private final VisionDeviceManager mVisionDevices = VisionDeviceManager.getInstance();

	// limelight
	private final Limelight mLimelight = Limelight.getInstance();

	// enabled and disabled loopers
	private final Looper mEnabledLooper = new Looper();
	private final Looper mDisabledLooper = new Looper();


  // subsystem instances
	private CommandSwerveDrivetrain mDrive;
  private IntakeSubsystem mIntakeRollers;


  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity


  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric


  public static boolean is_red_alliance = false;

  double disable_enter_time = 0.0;

  public Robot() {
		CrashTracker.logRobotConstruction();
	}
  
  @Override
  public void robotInit() {
    
    try 
    {
      //  m_robotContainer = new RobotContainer(); //L

        mIntakeRollers = IntakeSubsystem.getInstance();

        mDrive.setDefaultCommand( // Drivetrain will execute this command periodically
        mDrive.applyRequest(() -> drive.withVelocityX(-mControlBoard.driver.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-mControlBoard.driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-mControlBoard.driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));



        mDrive = CommandSwerveDrivetrain.getInstance();

        CrashTracker.logRobotInit();

			  LiveWindow.disableAllTelemetry();


        // spotless:off
			mSubsystemManager.setSubsystems(
				mIntakeRollers,
				mVisionDevices,
				mLimelight
			);
			// spotless:on


        mSubsystemManager.registerEnabledLoops(mEnabledLooper);
			  mSubsystemManager.registerDisabledLoops(mDisabledLooper);

        RobotController.setBrownoutVoltage(5.5);

			  DataLogManager.start();

    } catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
  }

  //This function is called after the autonomousPeriodic() or teleopPeriodic() functions are called and before any dashboard or LiveWindow updates.
  //A lot of teams had code that wrote variables to the dashboard in both autonomousPeriodic and teleopPeriodic, and robotPeriodic was added to allow only writing that code in one place.

  @Override
  public void robotPeriodic() {
    
    CommandScheduler.getInstance().run(); //L



    mEnabledLooper.outputToSmartDashboard();
		mSubsystemManager.outputLoopTimes();
  }


  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    /*m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }*/
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {


    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }


    try {
			RobotState.getInstance().setIsInAuto(false);
			VisionDeviceManager.setDisableVision(false);
			mDisabledLooper.stop();
			mEnabledLooper.start();

			mLimelight.setPipeline(Pipeline.TELEOP);
      
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
  }

  @Override
  public void teleopPeriodic() {

      try {

        mControlBoard.update();

        //TODO: Test this out - we might not need the command scheduler for this.
        /*
         mDrive.applyRequest(() -> drive.withVelocityX(-mControlBoard.driver.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-mControlBoard.driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-mControlBoard.driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ) 
         
      
        */
			mDriverControls.oneControllerMode();

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
  }

  @Override
	public void disabledInit() {
		try {
			VisionDeviceManager.setDisableVision(false);
			CrashTracker.logDisabledInit();
			mEnabledLooper.stop();
			mDisabledLooper.start();
			disable_enter_time = Timer.getFPGATimestamp();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}

		mLimelight.setPipeline(is_red_alliance ? Pipeline.AUTO_RED : Pipeline.AUTO_BLUE);
	}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
