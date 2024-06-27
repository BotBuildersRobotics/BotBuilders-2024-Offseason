package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.math.Matrix;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;
import frc.robot.Robot;

public class CameraIOPhoton implements CameraIO{
    
    protected final PhotonCamera camera;
    protected final PhotonPoseEstimator estimator;
    public AprilTagFieldLayout kTagLayout = (AprilTagFields.kDefaultField.loadAprilTagLayoutField());
    protected Transform3d robotToCam;
    private double lastEstTimestamp = 0;

    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);


    public CameraIOPhoton(){

        camera = new PhotonCamera("photonvision");

        robotToCam = Constants.CameraConstants.robotToCamera;
        estimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCam);
        estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    }

    @Override
    public void updateInputs(CameraIOInputs inputs) {
        // Query the latest result from PhotonVision
       /*  var visionEst = getEstimatedGlobalPose();

        if(visionEst != null){

            inputs.robotPose = visionEst.get().estimatedPose.toPose2d();
            inputs.timestamp = camera.getLatestResult().getTimestampSeconds();
           
            //can't log std dev
           // var estStdDevs = getEstimationStdDevs(visionEst.get().estimatedPose.toPose2d());
           // inputs.stdDevs = estStdDevs;
        }*/

    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        var visionEst = estimator.update();
        double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
       
        if (newResult) lastEstTimestamp = latestTimestamp;
        return visionEst;
    }

    public PhotonPipelineResult getLatestResult() {
        return camera.getLatestResult();
    }

    /**
     * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
     * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
     * This should only be used when there are targets visible.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     */
    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
        var estStdDevs = kSingleTagStdDevs;
        var targets = getLatestResult().getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = estimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist +=
                    tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }

    public PoseCalcResult getEstimatedPose(){

            var result = new PoseCalcResult();
            var robotPose = getEstimatedGlobalPose();
            if(robotPose.isPresent()){
                result.estimationStdDevs = getEstimationStdDevs(robotPose.get().estimatedPose.toPose2d());
                result.robotPose = robotPose.get().estimatedPose.toPose2d();
                result.timestamp = camera.getLatestResult().getTimestampSeconds();
                result.found = true;
            }else{
                result.found = false;
            }
            return result;

    }
}
