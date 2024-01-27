package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;
import frc.robot.vision.VisionIO.VisionIOInputs;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.stream.Collectors;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision {

  private final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  private final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  private final VisionIO[] io;
  private final VisionIOInputs[] m_inputs;
  private final PhotonPoseEstimator[] m_poseEstimators;
  private VisionSystemSim m_visionSimSystem = null;
  private final AprilTagFieldLayout kTagLayout =
      AprilTagFields.kDefaultField.loadAprilTagLayoutField();
  private Consumer<VisionUpdate> m_visionUpdateConsumer = null;

  public Vision(VisionIO... io) {
    this.io = io;
    m_inputs = new VisionIOInputs[io.length];
    m_poseEstimators = new PhotonPoseEstimator[io.length];
    for (int i = 0; i < io.length; i++) {
      m_inputs[i] = new VisionIOInputs();
      io[i].updateInputs(m_inputs[i]); // This sets camera names and transforms
      m_poseEstimators[i] =
          new PhotonPoseEstimator(
              kTagLayout,
              PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
              new PhotonCamera(m_inputs[i].cameraName),
              m_inputs[i].robotToCamera);
    }
    if (Constants.kIsSim) {
      m_visionSimSystem = new VisionSystemSim("sim_vision_system");
      m_visionSimSystem.addAprilTags(kTagLayout);
      for (int index = 0; index < io.length; index++) {
        PhotonCameraSim cameraSim =
            new PhotonCameraSim(
                new PhotonCamera(m_inputs[index].cameraName), getSimCameraProperties());
        cameraSim.enableDrawWireframe(true);
        m_visionSimSystem.addCamera(cameraSim, m_inputs[index].robotToCamera);
      }
    }
  }

  public void periodic() {
    for (int index = 0; index < io.length; index++) {
      io[index].updateInputs(m_inputs[index]);
      Logger.processInputs("Vision/Cam" + Integer.toString(index), m_inputs[index]);
    }

    final List<Pose2d> posesToLog = new ArrayList<>();
    final List<Pose3d> tagPosesToLog = new ArrayList<>();
    for (int i = 0; i < io.length; i++) {
      for (PhotonPipelineResult result : m_inputs[i].results) {
        Optional<EstimatedRobotPose> poseOptional = m_poseEstimators[i].update(result);
        poseOptional.ifPresent(
            pose -> {
              Pose2d estimatedPose = pose.estimatedPose.toPose2d();
              List<PhotonTrackedTarget> targets = result.getTargets();
              targets.forEach(
                  target -> {
                    // if (target.getPoseAmbiguity() >= 0.2) return;
                    kTagLayout
                        .getTagPose(target.getFiducialId())
                        .ifPresent(tagPose -> tagPosesToLog.add(tagPose));
                  });
              posesToLog.add(estimatedPose);
              m_visionUpdateConsumer.accept(
                  new VisionUpdate(
                      estimatedPose,
                      result.getTimestampSeconds(),
                      getEstimationStdDevs(estimatedPose, targets)));
            });
      }
    }
    Logger.recordOutput("Vision/EstimatedPoses", posesToLog.toArray(new Pose2d[posesToLog.size()]));
    Logger.recordOutput("Vision/TagPoses", tagPosesToLog.toArray(new Pose3d[tagPosesToLog.size()]));
    Logger.recordOutput(
        "Vision/TagPoses2D",
        tagPosesToLog.stream()
            .map(p -> p.toPose2d())
            .collect(Collectors.toList())
            .toArray(new Pose2d[tagPosesToLog.size()]));
  }

  public Matrix<N3, N1> getEstimationStdDevs(
      Pose2d estimatedPose, List<PhotonTrackedTarget> targets) {
    var estimatedStdDevs = kSingleTagStdDevs;
    int numTags = 0;
    double avgDist = 0;
    for (var target : targets) {
      var tagPose = kTagLayout.getTagPose(target.getFiducialId());
      if (tagPose.isEmpty()) continue;
      numTags++;
      avgDist +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }
    if (numTags == 0) return estimatedStdDevs;
    avgDist /= numTags;
    if (numTags > 1) estimatedStdDevs = kMultiTagStdDevs;
    // After 4 meters we can't trust vision
    if (numTags == 1 && avgDist > 4)
      estimatedStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    else estimatedStdDevs = estimatedStdDevs.times(1 + (avgDist * avgDist / 30));
    return estimatedStdDevs;
  }

  private SimCameraProperties getSimCameraProperties() {
    final SimCameraProperties cameraProperties = new SimCameraProperties();
    cameraProperties.setCalibration(1600, 1200, Rotation2d.fromDegrees(75));
    cameraProperties.setCalibError(0.35, 0.10);
    cameraProperties.setFPS(20);
    cameraProperties.setAvgLatencyMs(50);
    cameraProperties.setLatencyStdDevMs(15);
    return cameraProperties;
  }

  public void updateSim(Pose2d currentPose) {
    m_visionSimSystem.update(currentPose);
  }

  public void setVisionPoseConsumer(Consumer<VisionUpdate> consumer) {
    m_visionUpdateConsumer = consumer;
  }

  public static record VisionUpdate(
      Pose2d pose, double timestampSeconds, Matrix<N3, N1> standardDeviations) {}
}
