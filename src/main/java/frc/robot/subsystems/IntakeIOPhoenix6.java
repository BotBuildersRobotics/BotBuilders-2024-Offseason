package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.lib.TalonFXFactory;
import frc.robot.lib.TalonUtil;

public class IntakeIOPhoenix6 implements IntakeIO{
    
    private TalonFX frontRoller;
    private TalonFX rearRoller;

    private TalonFX feederRoller;

    private final DigitalInput beamBreakSensor;

    private final VoltageOut frontRequest = new VoltageOut(0.0, true, false, false, false);
    private final VoltageOut rearRequest = new VoltageOut(0.0, true, false, false, false);

    private final VoltageOut feederRequest = new VoltageOut(0.0, true, false, false, false);

    private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);

    public IntakeIOPhoenix6() {

    frontRoller = TalonFXFactory.createDefaultTalon(Ports.INTAKE_TOP_ROLLER);
    TalonUtil.applyAndCheckConfiguration(frontRoller, Constants.IntakeConstants.IntakeFXConfig());
    frontRoller.setInverted(false);

    rearRoller = TalonFXFactory.createDefaultTalon(Ports.INTAKE_BOTTOM_ROLLER);
    TalonUtil.applyAndCheckConfiguration(rearRoller, Constants.IntakeConstants.IntakeFXConfig());
    rearRoller.setInverted(false);
    rearRoller.setNeutralMode(NeutralModeValue.Coast);


    feederRoller = TalonFXFactory.createDefaultTalon(Ports.INTAKE_FEEDER_ROLLER);
    TalonUtil.applyAndCheckConfiguration(feederRoller, Constants.IntakeConstants.IntakeFXConfig());
    feederRoller.setInverted(false);
    feederRoller.setNeutralMode(NeutralModeValue.Coast);

    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicAcceleration =48;
    motionMagicConfigs.MotionMagicCruiseVelocity = 48;
    motionMagicConfigs.MotionMagicJerk = 0;

    feederRoller.getConfigurator().apply(motionMagicConfigs);

    beamBreakSensor = new DigitalInput(Ports.INTAKE_BEAMBREAK);

  }


   @Override
    public void setFrontMotorVoltage(double voltage) {
        frontRoller.setControl(frontRequest.withOutput(MathUtil.clamp(voltage, -12.0, 12.0)));
    }

    @Override
    public void setRearMotorVoltage(double voltage) {
        rearRoller.setControl(rearRequest.withOutput(MathUtil.clamp(voltage, -12.0, 12.0)));
    }

    @Override
    public void setFeederMotorVoltage(double voltage) {
        feederRoller.setControl(feederRequest.withOutput(MathUtil.clamp(voltage, -12.0, 12.0)));
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.beamBreakTripped = !beamBreakSensor.get();
    }

    @Override
    public void MoveFeederRotations(int rotations){
       
         feederRoller.setControl(new MotionMagicVoltage(rotations));
    }

    @Override
    public void RunFeederVoltage(int voltage){
         feederRoller.setControl(feederRequest.withOutput(MathUtil.clamp(voltage, -12.0, 12.0)));
    }

    @Override
    public void RunCounterVoltage(int voltage){
        rearRoller.setControl(rearRequest.withOutput(MathUtil.clamp(voltage, -12.0, 12.0)));
    }

}
