package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.InterpolatingDouble;
import frc.robot.lib.RegressionMap;
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
    private CameraSubsystem camera = CameraSubsystem.getInstance();

    private CommandSwerveDrivetrain swerveDriveTrain;

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
        WEAK,
        PASS,
        LONG,
        SPEAKER,
        AUTO
    }

    public enum SuperState {
        IDLE,
        AMP_SHOT,
        WEAK_SHOT,
        SPEAKER_SHOT,
        SUBWOOFER_SHOT,
        THIRTYFIVE_SHOT,
        THIRTYSIX_SHOT,
        LONG_PASS_SHOT,
        AUTO_AIM_SHOOTER,
        STOW_PIVOT,
        MANUAL_SHOT,
        AUTO_SHOT,
        INTAKE,
        INTAKE_COMPLETE,
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

        //log and update robot pose
        var poseResult = camera.getPoseCalculation();

        if(poseResult != null && poseResult.found){
            this.swerveDriveTrain.addVisionMeasurement(poseResult.robotPose, poseResult.timestamp, poseResult.estimationStdDevs);
        }

    }

    private SuperState handleStateTransitions() 
    {

        previousSuperState = currentSuperState;
        switch (wantedSuperState) {

            case OUTTAKE:
                currentSuperState = SuperState.OUTTAKE;
                break;
            case AUTO_SHOT:
                currentSuperState = SuperState.AUTO_SHOT;
                break;
            case CONTROLLED_SHOT:
                currentSuperState = SuperState.CONTROLLED_SHOT;
                break;
            case AMP_SHOT:
                currentSuperState = SuperState.AMP_SHOT;
                break;
            case WEAK_SHOT:
                currentSuperState = SuperState.WEAK_SHOT;
                break;
             case SPEAKER_SHOT:
                currentSuperState = SuperState.SPEAKER_SHOT;
                break;
            case FEEDING:
                currentSuperState = SuperState.FEEDING;
                break;
            case INTAKE_COMPLETE:
                currentSuperState = SuperState.INTAKE_COMPLETE;
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
            case THIRTYFIVE_SHOT:
                 currentSuperState = SuperState.THIRTYFIVE_SHOT;
                 break;
            case THIRTYSIX_SHOT:
                 currentSuperState = SuperState.THIRTYSIX_SHOT;
                 break;
            case LONG_PASS_SHOT:
                 currentSuperState = SuperState.LONG_PASS_SHOT;
                 break;
            case INTAKE:
                currentSuperState = intake.getCurrentState() == IntakeSystemState.INTAKE_COMPLETE ? SuperState.INTAKE_COMPLETE : SuperState.INTAKE;   
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
            case AUTO_SHOT:
                handleAutoShot();
                break;
            case INTAKE_COMPLETE: //this is when the note is at the beam break
                handleCompleteIntake();
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
            case WEAK_SHOT:
                handleWeakShot();
                break;
            case SPEAKER_SHOT:
                handleSpeakerShot();
                break;

            case SUBWOOFER_SHOT:
                handleSubwooferShot();
                break;

            case THIRTYFIVE_SHOT:
                handleThirtyFiveShot();
                break;

            case THIRTYSIX_SHOT:
                handleThirtySixShot();
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

    private void handleAutoShot(){
        double targetMetres = vision.getHorizontalDistanceToTargetMeters();
        double visionAngle = RegressionMap.kPivotAutoAimMap.getInterpolated(new InterpolatingDouble(targetMetres)).value;
        //do a lookup in our lookup Table, interoplate as needed
        SmartDashboard.putNumber("target M",targetMetres);
        SmartDashboard.putNumber("vision A",visionAngle);
        shotPrep = ShotPrepType.AUTO;
        pivot.setAngle(visionAngle);
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

    private void handleWeakShot(){
       // leds.setStrobeState(LightState.RED);
        pivot.setWantedState(PivotSystemState.SUBWOOFER);
        shotPrep = ShotPrepType.WEAK;
       
    }


    public void handleSubwooferShot(){
       // leds.setStrobeState(LightState.RAINBOW);
        pivot.setWantedState(PivotSystemState.AMP);
        shotPrep = ShotPrepType.SUB;
    }

    public void handleThirtyFiveShot(){
       // leds.setStrobeState(LightState.RAINBOW);
        pivot.setWantedState(PivotSystemState.THIRTYFIVE);
        shotPrep = ShotPrepType.SUB;
    }

    public void handleThirtySixShot(){
       // leds.setStrobeState(LightState.RAINBOW);
        pivot.setWantedState(PivotSystemState.THIRTYSIX);
        shotPrep = ShotPrepType.SUB;
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
        if(angle >= pivot.getPivotAngle()){//getCurrentPosition()){
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
        else if(this.shotPrep == ShotPrepType.WEAK){
             shooter.setWantedState(ShooterSystemState.WEAK);
        }
        else if(this.shotPrep == ShotPrepType.AUTO)
        {
            double targetMetres = vision.getHorizontalDistanceToTargetMeters();
            double topRollerVoltage = RegressionMap.kTopRollerAutoAimMap.getInterpolated(new InterpolatingDouble(targetMetres)).value;
            double bottomRollerVoltage = RegressionMap.kBottomRollerAutoAimMap.getInterpolated(new InterpolatingDouble(targetMetres)).value;

            shooter.setVoltages(topRollerVoltage,bottomRollerVoltage );
            shooter.setWantedState(ShooterSystemState.CUSTOM);
        }
        else
        {
            shooter.setWantedState(ShooterSystemState.LONG_SHOT);
        }
        //intake.setWantedState(IntakeSystemState.FEEDING);

    }

    private void handleFeeding(){
        if(this.shotPrep == ShotPrepType.AMP){
            intake.setWantedState(IntakeSystemState.AMPFEEDING);
        } else {
            intake.setWantedState(IntakeSystemState.FEEDING);
        }
    }

    private void handleManualShot(){
        shooter.setWantedState(ShooterSystemState.SHOOT);
    }

    private void handleCompleteIntake()
    {
        leds.setState(LightState.GREEN);
        
        
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

        //if the intake is at any other stage, lets make the lights green.
        if(intake.getCurrentState() == IntakeSystemState.INTAKE_COMPLETE || intake.getCurrentState() == IntakeSystemState.SHUFFLE){
             leds.setState(LightState.GREEN);
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

    public void setDriveTrain(CommandSwerveDrivetrain dt){
        this.swerveDriveTrain = dt;
    }

    public void addVisionPoseEstimate(Pose2d pose){
        if(this.swerveDriveTrain != null){
            this.swerveDriveTrain.addVisionMeasurement(pose, Timer.getFPGATimestamp());
        }
    }

}
