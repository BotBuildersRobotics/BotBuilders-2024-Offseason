package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterSystemState;

public class ShooterAmpCommand extends Command {

    private final ShooterSubsystem shooterSubSystem;

    public ShooterAmpCommand(ShooterSubsystem subsystem) {
        shooterSubSystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        shooterSubSystem.setWantedState(ShooterSystemState.AMP);
    }
    
}