package frc.robot.subsystems;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
    default void updateInputs(PivotIOInputs inputs) {}

    @AutoLog
    class PivotIOInputs {
       
        public boolean leftMotorConnected =false;
        public boolean rightMotorConnected=false;
        public double leftTemperature = 0.0;
        public double rightTemperature = 0.0;
        public double leftVelocityRPS = 0.0;
        public double rightVelocityRPS = 0.0;
        public double setpoint = 0.0;
        
        public double leftPosition = 0.0;
        public double rightPosition = 0.0;

        public double leftVoltage = 0.0;
        public double leftCurrent = 0.0;
        public double rightVoltage = 0.0;
        public double rightCurrent = 0.0;
        
    }

    default void setPivotAngle(double degrees) {}
    default double getCurrentPosition() {return 0.0;}

   
}
