package frc.robot.commands;

import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;


public class PivotCommand extends Command {
 
  private final PivotSubsystem pivotSubSystem;
  private final IntSupplier wantedDegrees;

  public PivotCommand(PivotSubsystem subsystem, IntSupplier degrees) {
      pivotSubSystem = subsystem;
      wantedDegrees = degrees;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    pivotSubSystem.setAngle(wantedDegrees.getAsInt());
  }

}
