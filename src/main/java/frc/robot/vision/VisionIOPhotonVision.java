package frc.robot.vision;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.RawSubscriber;
import edu.wpi.first.networktables.TimestampedRaw;
import edu.wpi.first.wpilibj.Timer;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOPhotonVision implements VisionIO {

  private static final double kDisconnectedTimeout = 0.5;
  private RawSubscriber m_photonDataSubscriber;
  private final Timer m_disconnectedTimer = new Timer();
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
    inputs.results = new PhotonPipelineResult[dataQueue.length];

    for (int index = 0; index < dataQueue.length; index++) {
      Packet dataPacket = new Packet(1);
      dataPacket.setData(dataQueue[index].value);
      if (dataPacket.getSize() < 1) {
        throw new NullPointerException("Data packet is empty. This should NEVER happen.");
      }
      PhotonPipelineResult result = PhotonPipelineResult.serde.unpack(dataPacket);
      double timestampSeconds =
          (dataQueue[index].timestamp / 1e6) - (result.getLatencyMillis() / 1e3);
      result.setTimestampSeconds(timestampSeconds);
      inputs.results[index] = result;
    }

    if (dataQueue.length > 0) {
      m_disconnectedTimer.reset();
    } else if (m_disconnectedTimer.hasElapsed(kDisconnectedTimeout)) {
      inputs.connected = false;
    } else {
      inputs.connected = true;
    }
  }
}
