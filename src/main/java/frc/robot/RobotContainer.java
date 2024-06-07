// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.IntSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Drive.AprilTagRotationSource;
import frc.robot.commands.IntakeIdleCommand;
import frc.robot.commands.IntakeOnCommand;
import frc.robot.commands.IntakeReverseCommand;
import frc.robot.commands.PivotCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ShooterIdleCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  private IntakeSubsystem intake = IntakeSubsystem.getInstance();
  private ShooterSubsystem shooter = ShooterSubsystem.getInstance();
  private PivotSubsystem pivot = PivotSubsystem.getInstance();

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.RobotCentric rotate = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
 
  private final AprilTagRotationSource aprilTagRotation = new AprilTagRotationSource();

  private final Telemetry logger = new Telemetry(MaxSpeed);

 // private Command autoRun = drivetrain.getAutoPath("Blue1");

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    joystick.rightTrigger()
				.whileTrue(new IntakeOnCommand(intake))
				.whileFalse(new IntakeIdleCommand(intake));


    //reverse intake
    joystick.leftTrigger()
        .whileTrue(new IntakeReverseCommand(intake))
        .whileFalse(new IntakeIdleCommand(intake));

    //use the d pad up button to drive the robot forward - just a test
    joystick.povUp().whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));

    IntSupplier degrees1 = new IntSupplier() {
     @Override
     public int getAsInt() {
         // TODO Auto-generated method stub
         return 0;
     }
    };

    IntSupplier degrees2 = new IntSupplier() {
     @Override
     public int getAsInt() {
         // TODO Auto-generated method stub
         return 30;
     }
    };

    joystick.povUpLeft().whileTrue(new PivotCommand(pivot, degrees1));

     joystick.povUpRight().whileTrue(new PivotCommand(pivot, degrees2));

    //b button will activate the limelite april tag lookup.
    joystick.b().whileTrue(drivetrain.applyRequest(() -> rotate.withRotationalRate(  aprilTagRotation.getRotation() *  MaxAngularRate)));
   
    joystick.a().whileTrue(new ShootCommand(shooter)).whileFalse(new ShooterIdleCommand(shooter));
    
    
    //joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
   /*joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));
    */
    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    configureBindings();

    //NamedCommands.registerCommand("MessageThing", Commands.print("Tests"));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No Auto");
  }
}
