package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

public class Photonvision extends SubsystemBase{
    static boolean enablePhotonInstances = true; //Nyahaha
    private final PhotonCamera camera;
    private PhotonPipelineResult pipelineResult;
    AprilTagFieldLayout aprilTagFieldLayout = null;
  	PhotonPoseEstimator photonPoseEstimator;

    public Photonvision() {
        camera = new PhotonCamera("Arducam_OV9281_USB_Camera");
        pipelineResult = new PhotonPipelineResult();
        /**
 *  Vision setup
 */
    try {
			aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      /*
      The camera relative to the robot
       */
			Transform3d robotToCam = new Transform3d(new Translation3d(0, -0.3, 0.71), new Rotation3d(0, Math.toRadians(16.0), Math.PI)); // TODO: find and fix!
			photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, this.getCamera(), robotToCam);

		} catch (IOException e) {
			DriverStation.reportError(e.toString(), true);
		}
    }

    public PhotonCamera getCamera() {
        return camera;
    }
    public boolean hasTargets() {
        return pipelineResult.hasTargets();
    }

    public List<PhotonTrackedTarget> targets() {
        return pipelineResult.targets;
    }

    public PhotonTrackedTarget getBestTarget() {
        return pipelineResult.getBestTarget();
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
      photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
      return photonPoseEstimator.update();
    }
    
    @Override
    public void periodic() {
        pipelineResult = camera.getLatestResult();
            	
    }

    public static void enableVision(boolean enable) {
        enablePhotonInstances = enable;
    }

}