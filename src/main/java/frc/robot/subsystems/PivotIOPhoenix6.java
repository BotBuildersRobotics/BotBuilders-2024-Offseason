package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Ports;
import frc.robot.lib.TalonFXFactory;

public class PivotIOPhoenix6 implements PivotIO{
    
    private final TalonFX leader;
    private final TalonFX follower;

    private TalonFXConfigurator leaderConfigurator;
    private TalonFXConfigurator followerConfigurator;


    private final CurrentLimitsConfigs currentLimitsConfigs;
    private final MotorOutputConfigs leaderMotorConfigs;
    private final MotorOutputConfigs followerMotorConfigs;
    private final SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs;
    private final Slot0Configs slot0Configs;
    private final MotionMagicConfigs motionMagicConfigs;
    private double setpoint;

    /* Status Signals */
    private StatusSignal<Double> supplyLeft;
    private StatusSignal<Double> supplyRight;
    private StatusSignal<Double> closedLoopReferenceSlope;
    double prevClosedLoopReferenceSlope = 0.0;
    double prevReferenceSlopeTimestamp = 0.0;

    public PivotIOPhoenix6 (){

        leader = TalonFXFactory.createDefaultTalon(Ports.PIVOT_MAIN);
        follower = TalonFXFactory.createDefaultTalon(Ports.PIVOT_FOLLOWER);

        this.leaderConfigurator = leader.getConfigurator();
        this.followerConfigurator = follower.getConfigurator();

        /* Create configs */
        currentLimitsConfigs = new CurrentLimitsConfigs();
        currentLimitsConfigs.StatorCurrentLimitEnable = true;
        currentLimitsConfigs.StatorCurrentLimit = 40.0;
        currentLimitsConfigs.SupplyCurrentLimit = 40.0;
        currentLimitsConfigs.SupplyTimeThreshold = 1.5;

        leaderMotorConfigs = new MotorOutputConfigs();
        leaderMotorConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        leaderMotorConfigs.PeakForwardDutyCycle = 1.0;
        leaderMotorConfigs.PeakReverseDutyCycle = -1.0;
        leaderMotorConfigs.NeutralMode = NeutralModeValue.Brake;

        softwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs();
        softwareLimitSwitchConfigs.ReverseSoftLimitEnable = true;
        softwareLimitSwitchConfigs.ReverseSoftLimitThreshold = 0;
        softwareLimitSwitchConfigs.ForwardSoftLimitEnable = true;
        softwareLimitSwitchConfigs.ForwardSoftLimitThreshold = (6.6 * (42 / 4));
        

        followerMotorConfigs = new MotorOutputConfigs();
        followerMotorConfigs.PeakForwardDutyCycle = 1.0;
        followerMotorConfigs.PeakReverseDutyCycle = -1.0;
        followerMotorConfigs.NeutralMode = NeutralModeValue.Brake;

        slot0Configs = new Slot0Configs();
        slot0Configs.kA = 0;
        slot0Configs.kP = 0.3;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;
        slot0Configs.kS = 0.3;
        slot0Configs.kV = 0.12;

        motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicAcceleration = 140;
        motionMagicConfigs.MotionMagicCruiseVelocity = 140;
        motionMagicConfigs.MotionMagicJerk = 0;


        OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs();
        openLoopRampsConfigs.DutyCycleOpenLoopRampPeriod = 0.02;
        openLoopRampsConfigs.TorqueOpenLoopRampPeriod = 0.02;
        openLoopRampsConfigs.VoltageOpenLoopRampPeriod = 0.02;

        ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();
        closedLoopRampsConfigs.DutyCycleClosedLoopRampPeriod = 0.02;
        closedLoopRampsConfigs.TorqueClosedLoopRampPeriod = 0.02;
        closedLoopRampsConfigs.VoltageClosedLoopRampPeriod = 0.02;

        leaderConfigurator.apply(currentLimitsConfigs);
        leaderConfigurator.apply(leaderMotorConfigs);
        leaderConfigurator.apply(slot0Configs);
        leaderConfigurator.apply(motionMagicConfigs);
        leaderConfigurator.apply(openLoopRampsConfigs);
        leaderConfigurator.apply(closedLoopRampsConfigs);
        leaderConfigurator.apply(softwareLimitSwitchConfigs);

        followerConfigurator.apply(currentLimitsConfigs);
        followerConfigurator.apply(leaderMotorConfigs);
        followerConfigurator.apply(slot0Configs);
        followerConfigurator.apply(motionMagicConfigs);
        followerConfigurator.apply(openLoopRampsConfigs);
        followerConfigurator.apply(closedLoopRampsConfigs);
        followerConfigurator.apply(softwareLimitSwitchConfigs);

        supplyLeft = leader.getSupplyCurrent();
        supplyRight = follower.getSupplyCurrent();
        closedLoopReferenceSlope = leader.getClosedLoopReferenceSlope();

        BaseStatusSignal.setUpdateFrequencyForAll(
            100, supplyLeft, supplyRight, closedLoopReferenceSlope);


        follower.setControl(new Follower(Ports.PIVOT_MAIN.getDeviceNumber(), true));

    }

    public void enableBrakeMode(boolean enable) {
        leaderMotorConfigs.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        followerMotorConfigs.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    
        leaderConfigurator.apply(leaderMotorConfigs);
        followerConfigurator.apply(followerMotorConfigs);
    }

    private double degreesToRotations(double degrees) {
        return 6.6 * (degrees / 4); //teeth per movement.
    }

    public double rotationsToDegrees(double rotations){
        return 4.0 * (rotations / 6.6);
    }

    public void resetHeight(double newDegrees) {
        leader.setPosition(degreesToRotations(newDegrees));
    }

    

    public void incrementHeight(double height){
        setpoint += height;
        leader.setControl(new MotionMagicVoltage(degreesToRotations(setpoint)));
    }

    @Override
    public void setPivotAngle(double degrees) {
        //turn off
       
        if (!DriverStation.isEnabled()) {
            leader.setControl(new VoltageOut(0.0));
        return;
        }

        setpoint = degrees;
        leader.setControl(new MotionMagicVoltage(degreesToRotations(setpoint)));
    }
    
    @Override
    public double getCurrentPosition(){
        return rotationsToDegrees(leader.getPosition().getValueAsDouble());
    }

   
}
