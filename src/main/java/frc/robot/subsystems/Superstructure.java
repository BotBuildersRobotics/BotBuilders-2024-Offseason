package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.IntakeSubsystem.IntakeSystemState;
import frc.robot.subsystems.LightsSubsystem.LightState;
import frc.robot.subsystems.PivotSubsystem.PivotSystemState;
import frc.robot.subsystems.ShooterSubsystem.ShooterSystemState;

public class Superstructure extends SubsystemBase {

    private IntakeSubsystem intake = IntakeSubsystem.getInstance();
    private ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    private PivotSubsystem pivot = PivotSubsystem.getInstance();
    private LightsSubsystem leds = LightsSubsystem.getInstance();
    private VisionSubsystem vision = VisionSubsystem.getInstance();

    public static Superstructure mInstance;

    private ShotPrepType shotPrep = ShotPrepType.SUB;

	public static Superstructure getInstance() {

        //Rethink this for how advantage kit does 
		if (mInstance == null) {
			mInstance = new Superstructure();
		}
		return mInstance;
	}

    public enum ShotPrepType{
        AMP,
        SUB,
        PASS,
        LONG,
        SPEAKER
    }

    public enum SuperState {
        IDLE,
        AMP_SHOT,
        SPEAKER_SHOT,
        SUBWOOFER_SHOT,
        LONG_PASS_SHOT,
        AUTO_AIM_SHOOTER,
        STOW_PIVOT,
        MANUAL_SHOT,
        INTAKE,
        STAGE,
        SHUFFLE,
        FEEDING,
        OUTTAKE,
        CONTROLLED_SHOT,
        READY_FOR_SHOT,
        READY_FOR_AMP_SHOT,
        MOVE_PIVOT_TO_STOW,
        PREPARE_SUBWOOFER_SHOT,
        PREPARE_AMP_SHOT,
        SHOOT_SUBWOOFER_SHOT
    }

    

    private SuperState currentSuperState = SuperState.IDLE;
    private SuperState wantedSuperState = SuperState.IDLE;
    private SuperState previousSuperState;

    public Superstructure(){

    }

    @Override
    public void periodic() {


        currentSuperState = handleStateTransitions();
        SmartDashboard.putString("Super State", currentSuperState.toString());
        applyStates();

    }

    private SuperState handleStateTransitions() 
    {

        previousSuperState = currentSuperState;
        switch (wantedSuperState) {

            case OUTTAKE:
                currentSuperState = SuperState.OUTTAKE;
                break;
            case CONTROLLED_SHOT:
                currentSuperState = SuperState.CONTROLLED_SHOT;
                break;
            case AMP_SHOT:
                currentSuperState = SuperState.AMP_SHOT;
                break;
             case SPEAKER_SHOT:
                currentSuperState = SuperState.SPEAKER_SHOT;
                break;
            case SHUFFLE: 
                 currentSuperState = SuperState.SHUFFLE;
                break;
            case FEEDING:
                currentSuperState = SuperState.FEEDING;
                break;
            case STAGE:
                currentSuperState = SuperState.STAGE;
                break;
            case MANUAL_SHOT:
                currentSuperState = SuperState.MANUAL_SHOT;
                break;
            case READY_FOR_SHOT:
                currentSuperState = SuperState.READY_FOR_SHOT;
                break;
            case READY_FOR_AMP_SHOT:
                currentSuperState = SuperState.READY_FOR_AMP_SHOT;
                break;
            case MOVE_PIVOT_TO_STOW:
                currentSuperState = SuperState.MOVE_PIVOT_TO_STOW;
                break;
            case PREPARE_SUBWOOFER_SHOT:
                currentSuperState = SuperState.PREPARE_SUBWOOFER_SHOT;
                break;
            case SHOOT_SUBWOOFER_SHOT:
                currentSuperState = SuperState.PREPARE_SUBWOOFER_SHOT;
                break;
            case SUBWOOFER_SHOT:
                 currentSuperState = SuperState.SUBWOOFER_SHOT;
                 break;
            case LONG_PASS_SHOT:
                 currentSuperState = SuperState.LONG_PASS_SHOT;
                 break;
            case INTAKE:
               
                currentSuperState = intake.isBeamBreakTripped() ? SuperState.SHUFFLE : SuperState.INTAKE;
               
                break;
            case IDLE:
                default:
                    currentSuperState = SuperState.IDLE;
                    break;

        }

        return currentSuperState;
    }

    private void applyStates() {

        switch (currentSuperState) {
            case OUTTAKE:
                handleOuttake();
                break;
            case INTAKE:
                handleIntake();
                 break;

            case SHUFFLE:
                handleShuffle();
                break;
            case STAGE: //this is when the note is at the beam break
                handleStaged();
                break;
            case FEEDING: //this is us feeding the note into the shooter
                handleFeeding();
                break;
            case MANUAL_SHOT: //shoot with no system checks
                handleManualShot();
                break;
            case CONTROLLED_SHOT: //shoot when system is ready
                handleControlledShot();
                break;

            case READY_FOR_SHOT:
                handleReadyForShot();
                break;

            case AMP_SHOT:
                handleAmpShot();
                break;
            case SPEAKER_SHOT:
                handleSpeakerShot();
                break;

            case SUBWOOFER_SHOT:
                handleSubwooferShot();
                break;

            case READY_FOR_AMP_SHOT:
                handleReadyForAmp();
                break;
            case MOVE_PIVOT_TO_STOW:
                handleMovePivotToStow();
                break;

            case PREPARE_SUBWOOFER_SHOT:
                handlePrepareSubWooferShot();
                break;
            
            
            case IDLE:
                default:
                    handleIdle();
                    break;
        }

    }

    private void handlePrepareSubWooferShot()
    {
        //move the pivot to the correct location
        pivot.setWantedState(PivotSystemState.SPEAKER);
    }

    private void  handleMovePivotToStow(){
        //move the pivot to the stow position.
        pivot.setWantedState(PivotSystemState.STOW);
    }

    private void handleSpeakerShot(){
       // leds.setStrobeState(LightState.ORANGE);
        pivot.setWantedState(PivotSystemState.SPEAKER);
       shotPrep = ShotPrepType.SPEAKER;
    }

    private void handleAmpShot(){
       // leds.setStrobeState(LightState.RED);
        pivot.setWantedState(PivotSystemState.AMP);
        shotPrep = ShotPrepType.AMP;
       
    }

    public void handleSubwooferShot(){
       // leds.setStrobeState(LightState.RAINBOW);
        pivot.setWantedState(PivotSystemState.SUBWOOFER);
        shotPrep = ShotPrepType.SUB;
    }

    private void handleShuffle(){
        
    }

    private void handleReadyForAmp(){
       // leds.setStrobeState(LightState.RED);
        shooter.setWantedState(ShooterSystemState.AMP);
        intake.setWantedState(IntakeSystemState.FEEDING);
    }

    

    private boolean areSystemsReadyForShot(){
        boolean ready = false;

        if(shooter.atSpeakerSetpoint()){
            ready = true;
        }

        return ready;
    }

    private boolean areSystemsReadyForAmpShot(){
        boolean ready = false;

        if(shooter.atAmpSetpoint() && isPivotReadyForAmp()){ 
            ready = true;
        }

        return ready;
    }

    private boolean isPivotReadyForSubwoofer(){
        return true;
    }

    private boolean isPivotReadyForAmp(){
        return true;
    }

    public boolean isPivotAtAngle(double angle){
        if(angle >= pivot.getCurrentPosition()){
            return true;
        }
        else{
            return false;
        }
    }

    public boolean isShooterAtSpeed(){
        
        if(this.shotPrep == ShotPrepType.AMP){
            return shooter.atAmpSetpoint();
        }
        else{
            return shooter.atSpeakerSetpoint();
        }

       
    }

    private void handleReadyForShot(){
        
       // leds.setStrobeState(LightState.FIRE);

        if(this.shotPrep == ShotPrepType.AMP){
            shooter.setWantedState(ShooterSystemState.AMP);
        }
        else if(this.shotPrep == ShotPrepType.SUB){
             shooter.setWantedState(ShooterSystemState.SHOOT);
        }
        else{
            shooter.setWantedState(ShooterSystemState.LONG_SHOT);
        }
        //intake.setWantedState(IntakeSystemState.FEEDING);

    }

    private void handleFeeding(){
        intake.setWantedState(IntakeSystemState.FEEDING);
        
    }

    private void handleManualShot(){
        shooter.setWantedState(ShooterSystemState.SHOOT);
    }

    private void handleStaged()
    {
        leds.setState(LightState.GREEN);
        intake.setWantedState(IntakeSystemState.STAGED);
        
    }
    private void handleIdle(){
        leds.setState(LightState.BLUE);
        intake.setWantedState(IntakeSystemState.IDLE);
        shooter.setWantedState(ShooterSystemState.IDLE);
        pivot.setWantedState(PivotSystemState.STOW);

    }

    private void handleIntake(){
        leds.setState(LightState.BLUE);
        pivot.setWantedState(PivotSystemState.INTAKE);
        if(pivot.getPivotAngle() > 30){ //only start the intake once the pivot is greater 30 degrees
            intake.setWantedState(IntakeSystemState.INTAKE);
        }
    }

    private void handleOuttake() {
        //leds.setStrobeState(LightState.COLOR_FLOW_RED);
        intake.setWantedState(IntakeSystemState.REVERSE);
        shooter.setWantedState(ShooterSystemState.IDLE);
    }

    public void handleControlledShot(){
        shooter.setWantedState(ShooterSystemState.SHOOT);
      // intake.setWantedState(IntakeSystemState.FEEDING);
    }

    public SuperState getCurrentSuperState(){
        return this.currentSuperState;
    }


    public void setWantedSuperState(SuperState wantedSuperState) {
        this.wantedSuperState = wantedSuperState;
    }

    public Command setWantedSuperStateCommand(SuperState wantedSuperState) {
        return new InstantCommand(() -> setWantedSuperState(wantedSuperState));
    }

}
