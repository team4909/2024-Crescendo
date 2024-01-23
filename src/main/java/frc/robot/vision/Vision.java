package frc.robot.vision;

import frc.robot.Constants;
import org.photonvision.simulation.VisionSystemSim;

public class Vision {
  private VisionSystemSim m_visionSimSystem = null;

  public Vision() {
    if (Constants.kIsSim) {
      m_visionSimSystem = new VisionSystemSim("sim_vision_system");
    }

    // PhotonPoseEstimator m_poseEstimator = new PhotonPoseEstimator();
    // m_poseEstimator.setMultiTagFallbackStrategy(null);
  }
}
