package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;

public class WaitForPivotCheckCommand extends Command{
    
    
    public Superstructure superStructure = Superstructure.getInstance();

    private final Timer timer = new Timer();
    private static final double DURATION = 1;

    private boolean shooterIsAtSpeed = false;

    DoubleSupplier desiredAngle;

    public void WaitForPivotCheckCommand(){
         this.desiredAngle =()  -> 30;
    }
   
    public void WaitForPivotCheckCommand(double angle){

        this.desiredAngle =()  -> angle;
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        shooterIsAtSpeed = superStructure.isPivotAtAngle(desiredAngle.getAsDouble());
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
