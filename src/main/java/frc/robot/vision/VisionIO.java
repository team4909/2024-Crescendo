package frc.robot.vision;

import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.targeting.PhotonPipelineResult;

public interface VisionIO {

  public static class VisionIOInputs implements LoggableInputs {
    public String cameraName = "";
    public Transform3d robotToCamera = new Transform3d();
    public PhotonPipelineResult[] results = new PhotonPipelineResult[] {};
    public boolean connected = false;

    @Override
    public void toLog(LogTable table) {
      table.put("CameraName", cameraName);
      table.put("RobotToCamera", robotToCamera);
      table.put("FrameCount", results.length);
      for (int i = 0; i < results.length; i++) {
        table.put("Result/" + Integer.toString(i), results[i]);
      }
      table.put("Connected", connected);
    }

    @Override
    public void fromLog(LogTable table) {
      cameraName = table.get("CameraName", cameraName);
      robotToCamera = table.get("RobotToCamera", robotToCamera);
      int frameCount = (int) table.get("FrameCount", 0);
      results = new PhotonPipelineResult[frameCount];
      for (int i = 0; i < frameCount; i++) {
        results[i] = table.get("Frame/" + Integer.toString(i), new PhotonPipelineResult());
      }
      connected = table.get("Connected", false);
    }
  }

  public default void updateInputs(VisionIOInputs inputs) {}
}
