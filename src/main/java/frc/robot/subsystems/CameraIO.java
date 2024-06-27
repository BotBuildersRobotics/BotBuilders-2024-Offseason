package frc.robot.subsystems;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;


public interface CameraIO {
    
     default void updateInputs(CameraIOInputs inputs) {}

     @AutoLog
    public static class CameraIOInputs {

   
        public boolean poseDetected = false;
        public boolean hasTargets = false;
        public Pose2d robotPose = new Pose2d();
        public double timestamp = 0.0;
        
       
       
    }

    default Optional<EstimatedRobotPose> getEstimatedGlobalPose(){ return null;};
    default PoseCalcResult getEstimatedPose(){ return null;};
}
