package frc.robot.vision;

import static frc.robot.Constants.fieldLayout;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.PoseEstimation;
import frc.robot.vision.VisionIO.VisionIOInputs;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.stream.Collectors;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision {

  private static final double kTargetLogTimeSecs = 0.1;
  // #region Set these to false to disable vision measurements;
  private final boolean kTrustVisionXY = true;
  private final boolean kTrustVisionTheta = true;
  private final boolean kIgnoreVisionInSim = true;
  private final boolean kIgnoreVisionInAuto = true;
  // #endregion
  private final VisionIO[] io;
  private final VisionIOInputs[] m_inputs;
  private final PhotonPoseEstimator[] m_poseEstimators;
  private VisionSystemSim m_visionSimSystem = null;
  private Map<Integer, Double> m_lastDetectionTimeIds = new HashMap<>();
  private ArrayList<VisionUpdate> m_newVisionUpdates;
  private final double kXYStdDevCoefficient = 0.1;
  private final double kThetaStdDevCoefficient = 0.1;

  public Vision(VisionIO... io) {
    this.io = io;
    m_inputs = new VisionIOInputs[io.length];
    m_poseEstimators = new PhotonPoseEstimator[io.length];
    for (int i = 0; i < io.length; i++) {
      m_inputs[i] = new VisionIOInputs();
      io[i].updateInputs(
          m_inputs[i]); // This is for initializing camera names and offsets into inputs.
      m_poseEstimators[i] =
          new PhotonPoseEstimator(
              fieldLayout,
              PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
              new PhotonCamera(m_inputs[i].cameraName),
              m_inputs[i].robotToCamera);
    }
    if (Constants.kIsSim) {
      m_visionSimSystem = new VisionSystemSim("sim_vision_system");
      m_visionSimSystem.addAprilTags(fieldLayout);
      for (int index = 0; index < io.length; index++) {
        PhotonCameraSim cameraSim =
            new PhotonCameraSim(
                new PhotonCamera(m_inputs[index].cameraName), getSimCameraProperties());
        cameraSim.enableDrawWireframe(true);
        m_visionSimSystem.addCamera(cameraSim, m_inputs[index].robotToCamera);
      }
    }

    fieldLayout
        .getTags()
        .forEach(
            (AprilTag tag) -> {
              m_lastDetectionTimeIds.put(tag.ID, 0.0);
            });
  }

  public void periodic() {
    for (int index = 0; index < io.length; index++) {
      io[index].updateInputs(m_inputs[index]);
      Logger.processInputs("VisionInputs/Cam" + Integer.toString(index), m_inputs[index]);
    }

    m_newVisionUpdates = new ArrayList<>();
    final List<Pose3d> estimatedPosesToLog = new ArrayList<>();
    for (int ioIndex = 0; ioIndex < io.length; ioIndex++) {
      for (int resultIndex = 0; resultIndex < m_inputs[ioIndex].results.length; resultIndex++) {
        final byte[] rawResult = m_inputs[ioIndex].results[resultIndex];
        final Packet dataPacket = new Packet(1);
        dataPacket.setData(rawResult);
        if (dataPacket.getSize() < 1)
          DriverStation.reportError("Photonvision data packet is empty.", true);
        final PhotonPipelineResult result = PhotonPipelineResult.serde.unpack(dataPacket);
        final double latencyCompensatedTimestampSeconds =
            (m_inputs[ioIndex].timestampsMillis[resultIndex] / 1e6)
                - (result.getLatencyMillis() / 1e3);
        result.setTimestampSeconds(latencyCompensatedTimestampSeconds);
        final Optional<EstimatedRobotPose> poseOptional = m_poseEstimators[ioIndex].update(result);
        poseOptional.ifPresent(
            pose -> {
              Pose3d estimatedPose = pose.estimatedPose;
              List<PhotonTrackedTarget> targets = result.getTargets();
              targets.forEach(
                  target -> {
                    fieldLayout
                        .getTagPose(target.getFiducialId())
                        .ifPresent(
                            tagPose ->
                                m_lastDetectionTimeIds.put(
                                    target.getFiducialId(), Timer.getFPGATimestamp()));
                  });
              estimatedPosesToLog.add(
                  estimatedPose.plus(
                      new Transform3d(
                          new Translation3d(Units.inchesToMeters(2.5), 0.0, 0.0),
                          new Rotation3d())));
              m_newVisionUpdates.add(
                  new VisionUpdate(
                      estimatedPose.toPose2d(),
                      result.getTimestampSeconds(),
                      getEstimationStdDevs(estimatedPose.toPose2d(), targets)));
            });
      }

      // Sort vision updates so more recent ones are given higher weight.
      m_newVisionUpdates.stream()
          .sorted(Comparator.comparingDouble(VisionUpdate::timestampSeconds))
          .forEach(PoseEstimation.getInstance()::addVisionMeasurement);
    }

    Logger.recordOutput(
        "Vision/EstimatedPoses",
        estimatedPosesToLog.toArray(new Pose3d[estimatedPosesToLog.size()]));
    List<Pose3d> targetPose3ds = new ArrayList<>();
    for (Map.Entry<Integer, Double> detectionEntry : m_lastDetectionTimeIds.entrySet())
      if (Timer.getFPGATimestamp() - detectionEntry.getValue() < kTargetLogTimeSecs)
        fieldLayout.getTagPose(detectionEntry.getKey()).ifPresent(pose -> targetPose3ds.add(pose));

    Logger.recordOutput("Vision/TagPoses", targetPose3ds.toArray(new Pose3d[targetPose3ds.size()]));
    Logger.recordOutput(
        "Vision/TagPoses2D",
        targetPose3ds.stream()
            .map(pose3d -> pose3d.toPose2d())
            .collect(Collectors.toList())
            .toArray(new Pose2d[targetPose3ds.size()]));
  }

  public Matrix<N3, N1> getEstimationStdDevs(
      Pose2d estimatedPose, List<PhotonTrackedTarget> targets) {
    if ((Constants.kIsSim && kIgnoreVisionInSim)
        || (DriverStation.isAutonomous() && kIgnoreVisionInAuto))
      return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    int tagCount = 0;
    double totalDistance = 0;
    for (PhotonTrackedTarget target : targets) {
      Optional<Pose3d> tagPose = fieldLayout.getTagPose(target.getFiducialId());
      // Ignore tags whose ids are not in the field layout.
      if (tagPose.isEmpty()) continue;
      tagCount++;
      totalDistance +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }
    double averageDistance = totalDistance / tagCount;

    /**
     * Reject tags is there are no valid tags OR there is only one tag and it has a high pose
     * ambiguity OR there is only one tag and the tag is too far.
     */
    if ((tagCount == 0)
        || (tagCount == 1 && targets.get(0).getPoseAmbiguity() > 0.25)
        || (tagCount == 1 && averageDistance > 4))
      return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);

    double xyStdDev =
        kTrustVisionXY
            ? kXYStdDevCoefficient * Math.pow(averageDistance, 2) / tagCount
            : Double.MAX_VALUE;
    double thetaStdDev =
        kTrustVisionTheta
            ? kThetaStdDevCoefficient * Math.pow(averageDistance, 2) / tagCount
            : Double.MAX_VALUE;
    return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
  }

  private SimCameraProperties getSimCameraProperties() {
    final SimCameraProperties cameraProperties = new SimCameraProperties();
    cameraProperties.setCalibration(1280, 720, Rotation2d.fromDegrees(75));
    cameraProperties.setCalibError(0.35, 0.10);
    cameraProperties.setFPS(20);
    cameraProperties.setAvgLatencyMs(50);
    cameraProperties.setLatencyStdDevMs(15);
    return cameraProperties;
  }

  public void updateSim(Pose2d currentPose) {
    m_visionSimSystem.update(currentPose);
  }

  public static record VisionUpdate(
      Pose2d pose, double timestampSeconds, Matrix<N3, N1> standardDeviations) {}
}
