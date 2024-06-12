package frc.robot.subsystems;

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



    public IntakeIOPhoenix6() {

    frontRoller = TalonFXFactory.createDefaultTalon(Ports.INTAKE_TOP_ROLLER);
    TalonUtil.applyAndCheckConfiguration(frontRoller, Constants.IntakeConstants.IntakeFXConfig());
    frontRoller.setInverted(true);

    rearRoller = TalonFXFactory.createDefaultTalon(Ports.INTAKE_BOTTOM_ROLLER);
    TalonUtil.applyAndCheckConfiguration(rearRoller, Constants.IntakeConstants.IntakeFXConfig());
    rearRoller.setInverted(false);
    rearRoller.setNeutralMode(NeutralModeValue.Coast);


    feederRoller = TalonFXFactory.createDefaultTalon(Ports.INTAKE_FEEDER_ROLLER);
    TalonUtil.applyAndCheckConfiguration(feederRoller, Constants.IntakeConstants.IntakeFXConfig());
    feederRoller.setInverted(false);
    feederRoller.setNeutralMode(NeutralModeValue.Coast);

    beamBreakSensor = new DigitalInput(1);

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

}
