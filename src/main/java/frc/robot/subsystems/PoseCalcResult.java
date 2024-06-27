package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class PoseCalcResult {
    
    public Matrix<N3, N1> estimationStdDevs;
    public Pose2d robotPose;
    public double timestamp;
}
