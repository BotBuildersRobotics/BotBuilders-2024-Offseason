package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.lib.TalonFXFactory;
import frc.robot.lib.TalonUtil;

public class ShooterIOPhoenix6 implements ShooterIO {
    private static final int SLOT = 0;

    private final TalonFX mTopFX;
	private final TalonFX mBottomFX;

    private final VoltageOut topRequest = new VoltageOut(0.0, true, false, false, false);
    private final VoltageOut bottomRequest = new VoltageOut(0.0, true, false, false, false);

     private final VelocityVoltage leftShooterVelocityControl =
            new VelocityVoltage(0, 0, true, 0, SLOT, false, false, false);
    private final VelocityVoltage rightShooterVelocityControl =
            new VelocityVoltage(0, 0, true, 0, SLOT, false, false, false);

    private final MotionMagicVelocityVoltage mmTopRequest = new MotionMagicVelocityVoltage(0);
    private final MotionMagicVelocityVoltage mmRequest = new MotionMagicVelocityVoltage(0);

    private MotionMagicConfigs motionMagicConfigs;

    public ShooterIOPhoenix6()
    {
       
        mTopFX = TalonFXFactory.createDefaultTalon(Ports.SHOOTER_TOP);
        TalonUtil.applyAndCheckConfiguration(mTopFX, Constants.ShooterConstants.ShooterFXConfig());
        mTopFX.setInverted(false);
        
        

        mBottomFX = TalonFXFactory.createDefaultTalon(Ports.SHOOTER_BOTTOM);
        TalonUtil.applyAndCheckConfiguration(mBottomFX, Constants.ShooterConstants.ShooterFXConfig());
        mBottomFX.setInverted(true);
        

        motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicAcceleration = 80;
        motionMagicConfigs.MotionMagicCruiseVelocity = 0;
        motionMagicConfigs.MotionMagicJerk = 0;

        mTopFX.getConfigurator().apply(motionMagicConfigs);
        mBottomFX.getConfigurator().apply(motionMagicConfigs);

        this.setBottomMotorVoltage(0);
        this.setTopMotorVoltage(0);

    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.topMotorConnected = BaseStatusSignal.refreshAll(
                        mTopFX.getMotorVoltage(),
                        mTopFX.getSupplyCurrent(),
                        mTopFX.getDeviceTemp(),
                        mTopFX.getVelocity())
                .isOK();
        inputs.bottomMotorConnected = BaseStatusSignal.refreshAll(
                        mBottomFX.getMotorVoltage(),
                        mBottomFX.getSupplyCurrent(),
                        mBottomFX.getDeviceTemp(),
                        mBottomFX.getVelocity())
                .isOK();

        inputs.topVoltage = mTopFX.getMotorVoltage().getValueAsDouble();
        inputs.topCurrent = mTopFX.getSupplyCurrent().getValueAsDouble();
        inputs.topTemperature = mTopFX.getDeviceTemp().getValueAsDouble();
        inputs.topVelocityRPS = mTopFX.getVelocity().getValueAsDouble();
        inputs.bottomVoltage = mBottomFX.getMotorVoltage().getValueAsDouble();
        inputs.bottomCurrent = mBottomFX.getSupplyCurrent().getValueAsDouble();
        inputs.bottomTemperature = mBottomFX.getDeviceTemp().getValueAsDouble();
        inputs.bottomVelocityRPS = mBottomFX.getVelocity().getValueAsDouble();
    }

    @Override
    public void setTopMotorVoltage(double voltage) {
        mTopFX.setControl(topRequest.withOutput(MathUtil.clamp(voltage, -12.0, 12.0)));
    }

    @Override
    public void setBottomMotorVoltage(double voltage) {
        mBottomFX.setControl(bottomRequest.withOutput(MathUtil.clamp(voltage, -12.0, 12.0)));
    }

    @Override
    public void setTopMotorRPStage(double rps) {
        mTopFX.setControl(mmTopRequest.withVelocity(rps));
    }

    @Override
    public void setBottomMotorRPS(double rps) {
        mBottomFX.setControl(mmRequest.withVelocity(rps));
    }
    

    private final double SHOOTER_GEAR_RATIO = 1.3;

    @Override
    public void setRPM(double topTargetRPM, double bottomTargetRPM) {

        double topApplied = topTargetRPM * SHOOTER_GEAR_RATIO / 60.0;
        double bottomApplied = bottomTargetRPM * SHOOTER_GEAR_RATIO / 60.0;

        mTopFX.setControl(leftShooterVelocityControl.withVelocity(topApplied));
        mBottomFX.setControl(rightShooterVelocityControl.withVelocity(bottomApplied));
    }

}
