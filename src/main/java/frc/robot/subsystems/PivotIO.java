package frc.robot.subsystems;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
    default void updateInputs(PivotIOInputs inputs) {}

    @AutoLog
    class PivotIOInputs {
       
        
        public double leftTemperature = 0.0;
        public double rightTemperature = 0.0;
        public double leftVelocityRPS = 0.0;
        public double rightVelocityRPS = 0.0;
        public double setpoint = 0;
    }

    default void setPivotAngle(double degrees) {}
    default double getCurrentPosition() {return 0.0;}

   
}
