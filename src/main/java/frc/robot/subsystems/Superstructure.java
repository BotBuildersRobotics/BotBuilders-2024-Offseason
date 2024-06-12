package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {

    private IntakeSubsystem intake = IntakeSubsystem.getInstance();
    private ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    private PivotSubsystem pivot = PivotSubsystem.getInstance();
    private LightsSubsystem leds = LightsSubsystem.getInstance();

    public enum CurrentSuperState {
        IDLE,
        AMP_SHOT,
        AUTO_AIM_SHOOTER,
        STOW_PIVOT,
        MANUAL_SHOT
    }

    private CurrentSuperState currentSuperState = CurrentSuperState.IDLE;

    public Superstructure(){

    }

    @Override
    public void periodic() {

    }

}
