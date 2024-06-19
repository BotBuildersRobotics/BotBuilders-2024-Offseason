package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;

public class WaitForShooterCheckCommand extends Command{
    
    public Superstructure superStructure = Superstructure.getInstance();

    private final Timer timer = new Timer();
    private static final double DURATION = 0.5;

    private boolean shooterIsAtSpeed = false;

   
    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        shooterIsAtSpeed = superStructure.isShooterAtSpeed();
    }

    @Override
    public void end(boolean interrupted) {
       
        timer.stop();
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(DURATION) || shooterIsAtSpeed;
    }
}
