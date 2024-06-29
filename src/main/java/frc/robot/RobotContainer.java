// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Drive.AprilTagLateralSource;
import frc.robot.Drive.AprilTagRotationSource;
import frc.robot.commands.ControllerRumbleCommand;
import frc.robot.commands.WaitForPivotCheckCommand;
import frc.robot.commands.WaitForShooterCheckCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.OperatorDashboard;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeSystemState;
import frc.robot.subsystems.Superstructure.SuperState;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  //TODO: Fix
 // private IntakeSubsystem intake = IntakeSubsystem.getInstance();
  private ShooterSubsystem shooter = ShooterSubsystem.getInstance();
  private PivotSubsystem pivot = PivotSubsystem.getInstance();
  private LightsSubsystem lights = LightsSubsystem.getInstance();
  private IntakeSubsystem intake = IntakeSubsystem.getInstance();
  private VisionSubsystem vision = VisionSubsystem.getInstance();

  private Superstructure superstructure = Superstructure.getInstance();

  private final OperatorDashboard dashboard;

  private final LoggedDashboardChooser<Command> autoChooser;


  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driverControl = new CommandXboxController(0); //

  private final CommandXboxController operatorControl = new CommandXboxController(1); //

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.RobotCentric rotate = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
 
  private final SwerveRequest.RobotCentric lateralMovement = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
 

  private final AprilTagRotationSource aprilTagRotation = new AprilTagRotationSource();
  private final AprilTagLateralSource aprilTagLateral = new AprilTagLateralSource();

  private final Telemetry logger = new Telemetry(MaxSpeed);

 // private Command autoRun = drivetrain.getAutoPath("Blue1");

  private void configureBindings() {

    
   
    //DRIVE THE ROBOT
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driverControl.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-driverControl.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driverControl.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    
    //ACCEPT THE NOTE - RUMBLE WHEN IN
    operatorControl.rightTrigger()
      .onTrue(superstructure.setWantedSuperStateCommand(SuperState.INTAKE))
      .onFalse(superstructure.setWantedSuperStateCommand(SuperState.IDLE));

    /*operatorControl.rightTrigger()
      .onTrue(superstructure.setWantedSuperStateCommand(SuperState.INTAKE))
      .onFalse(superstructure.setWantedSuperStateCommand(SuperState.IDLE))
      .whileTrue
      (
        new RepeatCommand(
          new SequentialCommandGroup
          (
             new ConditionalCommand
            (
                new SequentialCommandGroup
                (
                   
                    new InstantCommand( () -> intake.RunFeederVoltage(-6)), //Run in reverse to help clear note jams
                    new InstantCommand(() -> intake.RunFrontRollerVoltage(-6)),
                    new InstantCommand(() -> intake.RunCounterVoltage(-6)),
                    new WaitCommand(0.5),  //run just for a small period of time
                    superstructure.setWantedSuperStateCommand(SuperState.STAGE)
                ),
                new SequentialCommandGroup
                (
                 // new WaitCommand(0.5),
                  superstructure.setWantedSuperStateCommand(SuperState.INTAKE)
                ),
                () -> intake.getCurrentState() == IntakeSystemState.STAGED
            ),
            new SequentialCommandGroup 
            (
                  //new WaitCommand(0.2),
                  new ControllerRumbleCommand
                  (
                        driverControl, 
                       () -> intake.getCurrentState() == IntakeSystemState.STAGED
                  ),
                  new ControllerRumbleCommand
                  (
                        operatorControl, 
                        () -> intake.getCurrentState() == IntakeSystemState.STAGED
                  )
            )
          )
        )
        );*/
        

    //SPIT OUT THE NOTE                          
    operatorControl.leftBumper()
       .onTrue(superstructure.setWantedSuperStateCommand(SuperState.OUTTAKE))
       .onFalse(superstructure.setWantedSuperStateCommand(SuperState.IDLE));

    //SHoot the staged shot
    operatorControl.leftTrigger().onTrue(
      new SequentialCommandGroup(
        superstructure.setWantedSuperStateCommand(SuperState.READY_FOR_SHOT),
        new WaitForShooterCheckCommand(),
        superstructure.setWantedSuperStateCommand(SuperState.FEEDING),
        new WaitCommand(0.5),
        superstructure.setWantedSuperStateCommand(SuperState.IDLE)
        
      )
    );

 
    operatorControl.a().onTrue(superstructure.setWantedSuperStateCommand(SuperState.AMP_SHOT));
    //operatorControl.b().onTrue(superstructure.setWantedSuperStateCommand(SuperState.SPEAKER_SHOT));
    operatorControl.y().onTrue(superstructure.setWantedSuperStateCommand(SuperState.SUBWOOFER_SHOT));
    operatorControl.x().onTrue(superstructure.setWantedSuperStateCommand(SuperState.LONG_PASS_SHOT)); 
    
    //treat the B button as auto aim.
    operatorControl.b().onTrue(superstructure.setWantedSuperStateCommand(SuperState.AUTO_SHOT));

    

    //Operator can manually STOW
    operatorControl.rightBumper().onTrue(
      superstructure.setWantedSuperStateCommand(SuperState.IDLE)
    );

    //Move the pivot up 
    operatorControl.povUp().onTrue(
            new InstantCommand(() -> pivot.setAngle(pivot.getCurrentPosition() + 2))

     );

     //Move the pivot down
     operatorControl.povDown().onTrue(
            new InstantCommand(() -> pivot.setAngle(pivot.getCurrentPosition() -2))

     );

    //DRIVER CAN ROTATE to aim at April Tag
    driverControl.b().whileTrue(drivetrain.applyRequest(() -> rotate.withRotationalRate(  aprilTagRotation.getRotation() *  MaxAngularRate)));
   
    
    //DRIVER CAN MOVE LATERALLY TO the AMP, based on April Tag

    driverControl.a().whileTrue(drivetrain.applyRequest(() -> lateralMovement.withVelocityX(  - aprilTagLateral.getLateral() *  1.1))); //TODO Tune

    // reset the field-centric heading on left bumper press
    driverControl.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {

    dashboard = new OperatorDashboard();

    configureBindings();

    

    NamedCommands.registerCommand("AutoShoot", 
      new SequentialCommandGroup(
        
        superstructure.setWantedSuperStateCommand(SuperState.SUBWOOFER_SHOT),
        new WaitForPivotCheckCommand(),
        superstructure.setWantedSuperStateCommand(SuperState.READY_FOR_SHOT),
        new WaitForShooterCheckCommand(),
        superstructure.setWantedSuperStateCommand(SuperState.FEEDING),
        new WaitCommand(0.5),
        superstructure.setWantedSuperStateCommand(SuperState.IDLE)
      )
    );

    NamedCommands.registerCommand("Intake", 
      new SequentialCommandGroup(
        
        superstructure.setWantedSuperStateCommand(SuperState.INTAKE)
      )
    );

    //Build the autos from pathplanner
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());


  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
   /*  return new SequentialCommandGroup(
        
        superstructure.setWantedSuperStateCommand(SuperState.SUBWOOFER_SHOT),
        //new WaitForPivotCheckCommand(),
        new WaitCommand(1),
        superstructure.setWantedSuperStateCommand(SuperState.READY_FOR_SHOT),
        new WaitForShooterCheckCommand(),
        superstructure.setWantedSuperStateCommand(SuperState.FEEDING),
        new WaitCommand(0.5),
        superstructure.setWantedSuperStateCommand(SuperState.IDLE)
      );*/
   // return Commands.print("No Auto");
  }
}
