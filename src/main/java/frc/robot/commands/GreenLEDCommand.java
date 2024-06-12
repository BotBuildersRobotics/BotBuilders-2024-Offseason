package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.LightsSubsystem.LightState;


public class GreenLEDCommand extends Command
{
  private final LightsSubsystem lightsSubSystem;

  public GreenLEDCommand(LightsSubsystem subsystem) {
      lightsSubSystem = subsystem;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    lightsSubSystem.setState(LightState.GREEN);
  }
}
