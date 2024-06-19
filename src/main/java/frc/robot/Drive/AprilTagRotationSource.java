package frc.robot.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AprilTagRotationSource implements RotationSource {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    public static PIDController rotationPID = createPIDController();
    private static PIDController createPIDController() {

        PIDController pid = new PIDController(.01, 0, 0);
        pid.setTolerance(.25); // allowable angle error
        pid.enableContinuousInput(0, 360); // it is faster to go 1 degree from 359 to 0 instead of 359 degrees
        pid.setSetpoint(0); // 0 = apriltag angle
        return pid;
    }
    
    @Override
    public double getRotation() {
        //the tx parameter has the rotation amount
        SmartDashboard.putNumber("limelight-d", table.getEntry("tx").getDouble(0));
        return rotationPID.calculate(table.getEntry("tx").getDouble(0));
    }
}
