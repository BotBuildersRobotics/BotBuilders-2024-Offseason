package frc.robot.subsystems;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    default void updateInputs(IntakeIOInputs inputs) {}

    @AutoLog
    class IntakeIOInputs {
        public boolean frontMotorConnected = true;
        public boolean rearMotorConnected = true;
        public boolean feederMotorConnected = true;
        public double frontVoltage = 0.0;
        public double rearVoltage = 0.0;
        public double frontCurrent = 0.0;
        public double rearCurrent = 0.0;
        public double frontTemperature = 0.0;
        public double rearTemperature = 0.0;
        public double frontVelocityRPS = 0.0;
        public double rearVelocityRPS = 0.0;


        public double feederCurrent = 0.0;
        public double feederTemperature = 0.0;
        public double feederVelocityRPS = 0.0;
        public double feederVoltage = 0.0;
        
        public boolean beamBreakTripped = false;
        

    }

    default void setFrontMotorVoltage(double voltage) {}

    default void setRearMotorVoltage(double voltage) {}

    default void setFeederMotorVoltage(double voltage) {}

    default void MoveFeederRotations(int rotations) {};

    default void RunFeederVoltage(int voltage){};

    default void RunCounterVoltage(int voltage){};

    default void RunFrontRollerVoltage(int voltage){};
}
