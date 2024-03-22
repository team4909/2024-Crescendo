package frc.robot.vision;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.RawSubscriber;
import edu.wpi.first.networktables.TimestampedRaw;
import edu.wpi.first.wpilibj.Timer;

public class VisionIOPhotonVision implements VisionIO {

  private final double kDisconnectedTimeoutSeconds = 0.5;
  private final Timer m_disconnectedTimer = new Timer();
  private RawSubscriber m_photonDataSubscriber;
  private final String m_cameraName;
  private final Transform3d m_robotToCamera;

  public VisionIOPhotonVision(String cameraName, Transform3d robotToCamera) {
    m_cameraName = cameraName;
    m_robotToCamera = robotToCamera;
    m_photonDataSubscriber =
        NetworkTableInstance.getDefault()
            .getTable("/photonvision/" + cameraName)
            .getRawTopic("rawBytes")
            .subscribe(
                "rawBytes",
                new byte[] {},
                PubSubOption.keepDuplicates(true),
                PubSubOption.sendAll(true));
  }

  public void updateInputs(VisionIOInputs inputs) {
    inputs.cameraName = m_cameraName;
    inputs.robotToCamera = m_robotToCamera;
    TimestampedRaw[] dataQueue = m_photonDataSubscriber.readQueue();
    inputs.results = new byte[dataQueue.length][];
    inputs.timestampsMillis = new double[dataQueue.length];

    for (int index = 0; index < dataQueue.length; index++) {
      inputs.results[index] = dataQueue[index].value;
      inputs.timestampsMillis[index] = dataQueue[index].timestamp;
    }

    if (dataQueue.length > 0) {
      inputs.connected = true;
      m_disconnectedTimer.reset();
    } else if (m_disconnectedTimer.hasElapsed(kDisconnectedTimeoutSeconds)) {
      inputs.connected = false;
    }
  }
}
