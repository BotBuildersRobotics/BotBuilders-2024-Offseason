package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeSystemState;



public class IntakeFeedCommand extends Command
{
  private final IntakeSubsystem intakeSubSystem;

  public IntakeFeedCommand(IntakeSubsystem subsystem) {
      intakeSubSystem = subsystem;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    intakeSubSystem.setWantedState(IntakeSystemState.FEEDING);
  }
}
