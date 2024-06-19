package frc.robot.subsystems;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    default void updateInputs(ShooterIOInputs inputs) {}

    @AutoLog
    class ShooterIOInputs {
        public boolean topMotorConnected = true;
        public boolean bottomMotorConnected = true;
        public double topVoltage = 0.0;
        public double bottomVoltage = 0.0;
        public double topCurrent = 0.0;
        public double bottomCurrent = 0.0;
        public double topTemperature = 0.0;
        public double bottomTemperature = 0.0;
        public double topVelocityRPS = 0.0;
        public double bottomVelocityRPS = 0.0;
    }

    default void setTopMotorVoltage(double voltage) {}

    default void setBottomMotorVoltage(double voltage) {}

    default void setTopMotorRPStage(double rps) {}

    default void setBottomMotorRPS(double rps) {}

    default void setRPM(double topTargetRPM, double bottomTargetRPM){}

    default boolean isShooterAmpReady(){return false;}

    default boolean isShooterReady(){return false;}
}
