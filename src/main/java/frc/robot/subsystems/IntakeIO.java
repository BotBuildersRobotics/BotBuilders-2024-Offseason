package frc.robot.subsystems;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    default void updateInputs(IntakeIOInputs inputs) {}

    @AutoLog
    class IntakeIOInputs {
        public boolean frontMotorConnected = true;
        public boolean rearMotorConnected = true;
        public double frontVoltage = 0.0;
        public double rearVoltage = 0.0;
        public double frontCurrent = 0.0;
        public double rearCurrent = 0.0;
        public double frontTemperature = 0.0;
        public double rearTemperature = 0.0;
        public double frontVelocityRPS = 0.0;
        public double rearVelocityRPS = 0.0;
        public boolean beamBreakTripped = false;
    }

    default void setFrontMotorVoltage(double voltage) {}

    default void setRearMotorVoltage(double voltage) {}

    default void setFeederMotorVoltage(double voltage) {}

    default void MoveRotations(int rotations) {};

    default void RunCounterSlow(int voltage){};
}
