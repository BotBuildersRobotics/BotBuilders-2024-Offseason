package frc.robot.subsystems;

import java.util.Random;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.lib.TalonFXFactory;
import frc.robot.lib.TalonUtil;
import frc.robot.subsystems.ShooterIO.ShooterIOInputs;

public class IntakeIOPhoenix6 implements IntakeIO{
    
    private TalonFX frontRoller;
    private TalonFX rearRoller;

    private TalonFX feederRoller;

    private final DigitalInput beamBreakSensor;
    private final DigitalInput beamBreakSensorTwo;

    

    private final VoltageOut frontRequest = new VoltageOut(0.0, true, false, false, false);
    private final VoltageOut rearRequest = new VoltageOut(0.0, true, false, false, false);

    private final VoltageOut feederRequest = new VoltageOut(0.0, true, false, false, false);

    private final MotionMagicVoltage mmVoltage = new MotionMagicVoltage(0);

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

    MotionMagicConfigs motionMagic = new MotionMagicConfigs();
    motionMagic.MotionMagicAcceleration = 48;
    motionMagic.MotionMagicCruiseVelocity = 48;

    feederRoller.getConfigurator().apply(motionMagic);

    beamBreakSensor = new DigitalInput(Ports.INTAKE_BEAMBREAK);
    beamBreakSensorTwo = new DigitalInput(Ports.INTAKE_BEAMBREAKTWO);

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
        inputs.beamBreakTwoTripped = !beamBreakSensorTwo.get();
        inputs.frontMotorConnected = BaseStatusSignal.refreshAll(
                        frontRoller.getMotorVoltage(),
                        frontRoller.getSupplyCurrent(),
                        frontRoller.getDeviceTemp(),
                        frontRoller.getVelocity())
                .isOK();
        inputs.rearMotorConnected = BaseStatusSignal.refreshAll(
                        rearRoller.getMotorVoltage(),
                        rearRoller.getSupplyCurrent(),
                        rearRoller.getDeviceTemp(),
                        rearRoller.getVelocity())
                .isOK();

        inputs.feederMotorConnected = BaseStatusSignal.refreshAll(
                        rearRoller.getMotorVoltage(),
                        rearRoller.getSupplyCurrent(),
                        rearRoller.getDeviceTemp(),
                        rearRoller.getVelocity())
                .isOK();

        inputs.rearVoltage = rearRoller.getMotorVoltage().getValueAsDouble();
        inputs.rearCurrent = rearRoller.getSupplyCurrent().getValueAsDouble();
        inputs.rearTemperature = rearRoller.getDeviceTemp().getValueAsDouble();
        inputs.rearVelocityRPS = rearRoller.getVelocity().getValueAsDouble();
        inputs.frontVoltage = frontRoller.getMotorVoltage().getValueAsDouble();
        inputs.frontCurrent = frontRoller.getSupplyCurrent().getValueAsDouble();
        inputs.frontTemperature = frontRoller.getDeviceTemp().getValueAsDouble();
        inputs.frontVelocityRPS = frontRoller.getVelocity().getValueAsDouble();

        inputs.feederVoltage = feederRoller.getMotorVoltage().getValueAsDouble();
        inputs.feederCurrent = feederRoller.getSupplyCurrent().getValueAsDouble();
        inputs.feederTemperature = feederRoller.getDeviceTemp().getValueAsDouble();
        inputs.feederVelocityRPS = feederRoller.getVelocity().getValueAsDouble();

    }

     @Override
    public void MoveFeederRotations(int rotations){
         Random rnd = new Random();
         SmartDashboard.putNumber("Rots", rotations + rnd.nextInt(5) );
         feederRoller.setPosition(0);
         feederRoller.setControl(mmVoltage.withPosition(rotations));
    }

    @Override
    public void RunFeederVoltage(int voltage){
         feederRoller.setControl(feederRequest.withOutput(MathUtil.clamp(voltage, -12.0, 12.0)));
    }

    @Override
    public void RunFrontRollerVoltage(int voltage){
        rearRoller.setControl(rearRequest.withOutput(MathUtil.clamp(voltage, -12.0, 12.0)));
    }

    @Override
    public void RunCounterVoltage(int voltage){
        rearRoller.setControl(rearRequest.withOutput(MathUtil.clamp(voltage, -12.0, 12.0)));
    }

}
