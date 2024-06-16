package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ControllerRumbleCommand extends Command {
    private CommandXboxController controller;
    private BooleanSupplier condition;

    public ControllerRumbleCommand(CommandXboxController controller, BooleanSupplier condition) {
        this.controller = controller;
        this.condition = condition;
    }

    @Override
    public void execute() {
        if (condition.getAsBoolean()) {
            controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.4);
        }
    }

    @Override
    public void end(boolean interrupted) {
        controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
    }
}