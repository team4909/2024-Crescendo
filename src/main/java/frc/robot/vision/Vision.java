package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.io.IOException;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.VisionSystemSim;

public class Vision extends SubsystemBase {
  private VisionSystemSim m_visionSimSystem = null;
  public PhotonCamera m_PiVision;
  private double skew = 0;
  private double yaw = 0;
  private double pitch = 0;

  private AprilTagFieldLayout aprilTagFieldLayout;
  private Transform3d robotToCam;
  private Transform3d cameraToRobot;
  private Pose2d m_pose;
  PhotonPoseEstimator photonPoseEstimator;
  Rotation2d m_Rotation;

  public Vision() {
    if (Constants.kIsSim) {
      m_visionSimSystem = new VisionSystemSim("sim_vision_system");
      m_Rotation = new Rotation2d(0, 0);
      m_PiVision = new PhotonCamera("Camera_Module_v1");
      m_pose = new Pose2d(5, 5, m_Rotation);
      try {
        aprilTagFieldLayout =
            AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      } catch (IOException e) {
        e.printStackTrace();
      }
      robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
      // cameraToRobot = new Transform3d(0, 0, 0, null);
      photonPoseEstimator =
          new PhotonPoseEstimator(
              aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, m_PiVision, robotToCam);
    }

    // PhotonPoseEstimator m_poseEstimator = new PhotonPoseEstimator();
    // m_poseEstimator.setMultiTagFallbackStrategy(null);
  }

  public void poseUpt() {
    getEstimatedRobotPose(m_pose).ifPresent((pose) -> m_pose = pose.estimatedPose.toPose2d());
    SmartDashboard.putNumber("X:", m_pose.getX());
    SmartDashboard.putNumber("Y:", m_pose.getY());
  }

  public Optional<EstimatedRobotPose> getEstimatedRobotPose(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator.update();
  }

  public void updateValues() {
    if (m_PiVision.getLatestResult().hasTargets()) {
      var res = m_PiVision.getLatestResult().getBestTarget();
      skew = res.getSkew();
      yaw = res.getYaw();
      pitch = res.getPitch();
      SmartDashboard.putNumber("yaw", yaw);
      SmartDashboard.putNumber("pitch", pitch);
    }
  }

  public double getYaw() {
    if (m_PiVision.getLatestResult().hasTargets()) {
      return m_PiVision.getLatestResult().getBestTarget().getYaw();
    } else return 0;
  }

  public boolean getIfTargets() {
    return m_PiVision.getLatestResult().hasTargets();
  }
}
