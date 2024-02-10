package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.PolynomialRegression;
import frc.robot.Constants;
import frc.robot.vision.VisionIO.VisionIOInputs;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
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

  private static final double kTargetLogTimeSecs = 0.1;
  private final VisionIO[] io;
  private final VisionIOInputs[] m_inputs;
  private final PhotonPoseEstimator[] m_poseEstimators;
  private VisionSystemSim m_visionSimSystem = null;
  private final AprilTagFieldLayout kTagLayout =
      AprilTagFields.kDefaultField.loadAprilTagLayoutField();
  private Consumer<VisionUpdate> m_visionUpdateConsumer = null;
  private Map<Integer, Double> m_lastDetectionTimeIds = new HashMap<>();
  private final PolynomialRegression xyStdDevModel =
      new PolynomialRegression(
          new double[] {
            0.752358, 1.016358, 1.296358, 1.574358, 1.913358, 2.184358, 2.493358, 2.758358,
            3.223358, 4.093358, 4.726358
          },
          new double[] {
            0.005, 0.0135, 0.016, 0.038, 0.0515, 0.0925, 0.0695, 0.046, 0.1245, 0.0815, 0.193
          },
          1);
  private final PolynomialRegression thetaStdDevModel =
      new PolynomialRegression(
          new double[] {
            0.752358, 1.016358, 1.296358, 1.574358, 1.913358, 2.184358, 2.493358, 2.758358,
            3.223358, 4.093358, 4.726358
          },
          new double[] {0.008, 0.027, 0.015, 0.044, 0.04, 0.078, 0.049, 0.027, 0.059, 0.029, 0.068},
          1);

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

    kTagLayout
        .getTags()
        .forEach(
            (AprilTag tag) -> {
              m_lastDetectionTimeIds.put(tag.ID, 0.0);
            });
  }

  public void periodic() {
    for (int index = 0; index < io.length; index++) {
      io[index].updateInputs(m_inputs[index]);
      Logger.processInputs("Vision/Cam" + Integer.toString(index), m_inputs[index]);
    }

    final List<Pose2d> robotPosesToLog = new ArrayList<>();
    for (int i = 0; i < io.length; i++) {
      for (PhotonPipelineResult result : m_inputs[i].results) {
        Optional<EstimatedRobotPose> poseOptional = m_poseEstimators[i].update(result);
        poseOptional.ifPresent(
            pose -> {
              Pose2d estimatedPose = pose.estimatedPose.toPose2d();
              List<PhotonTrackedTarget> targets = result.getTargets();
              targets.forEach(
                  target -> {
                    kTagLayout
                        .getTagPose(target.getFiducialId())
                        .ifPresent(
                            tagPose ->
                                m_lastDetectionTimeIds.put(
                                    target.getFiducialId(), Timer.getFPGATimestamp()));
                  });
              robotPosesToLog.add(estimatedPose);
              m_visionUpdateConsumer.accept(
                  new VisionUpdate(
                      estimatedPose,
                      result.getTimestampSeconds(),
                      getEstimationStdDevs(estimatedPose, targets)));
            });
      }
    }

    Logger.recordOutput(
        "Vision/EstimatedPoses", robotPosesToLog.toArray(new Pose2d[robotPosesToLog.size()]));
    List<Pose3d> targetPose3ds = new ArrayList<>();
    for (Map.Entry<Integer, Double> detectionEntry : m_lastDetectionTimeIds.entrySet())
      if (Timer.getFPGATimestamp() - detectionEntry.getValue() < kTargetLogTimeSecs)
        kTagLayout.getTagPose(detectionEntry.getKey()).ifPresent(pose -> targetPose3ds.add(pose));

    Logger.recordOutput("Vision/TagPoses", targetPose3ds.toArray(new Pose3d[targetPose3ds.size()]));
    Logger.recordOutput(
        "Vision/TagPoses2D",
        targetPose3ds.stream()
            .map(p -> p.toPose2d())
            .collect(Collectors.toList())
            .toArray(new Pose2d[targetPose3ds.size()]));
  }

  public Matrix<N3, N1> getEstimationStdDevs(
      Pose2d estimatedPose, List<PhotonTrackedTarget> targets) {
    Vector<N3> rejectTagStdDev =
        VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    int numTags = 0;
    double avgDist = 0;
    for (var target : targets) {
      var tagPose = kTagLayout.getTagPose(target.getFiducialId());
      // Ignore tags whose ids are not in the field layout.
      if (tagPose.isEmpty()) continue;
      numTags++;
      avgDist +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }
    if (numTags == 0) return rejectTagStdDev;
    if (numTags == 1) {
      if (targets.get(0).getPoseAmbiguity() > 0.3) {
        return rejectTagStdDev;
      }
    }
    avgDist /= numTags;
    // After 4 meters we can't trust vision
    if (numTags == 1 && avgDist > 4) return rejectTagStdDev;
    double xyStdDev = xyStdDevModel.predict(avgDist);
    double thetaStdDev = thetaStdDevModel.predict(avgDist);
    return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
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
