package frc.robot.vision;

import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface VisionIO {

  public static class VisionIOInputs implements LoggableInputs {
    public String cameraName = "";
    public Transform3d robotToCamera = new Transform3d();
    public double[] timestampsMillis = new double[] {};
    public byte[][] results = new byte[][] {};
    public boolean connected = false;

    @Override
    public void toLog(LogTable table) {
      table.put("CameraName", cameraName);
      table.put("RobotToCamera", robotToCamera);
      table.put("Timestamps", timestampsMillis);
      table.put("ResultCount", results.length);
      for (int i = 0; i < results.length; i++) {
        table.put("Result/" + Integer.toString(i), results[i]);
      }
      table.put("Connected", connected);
    }

    @Override
    public void fromLog(LogTable table) {
      cameraName = table.get("CameraName", cameraName);
      robotToCamera = table.get("RobotToCamera", robotToCamera);
      timestampsMillis = table.get("Timestamps", new double[] {0.0});
      int resultCount = table.get("ResultCount", 0);
      results = new byte[resultCount][];
      for (int i = 0; i < resultCount; i++) {
        results[i] = table.get("Result/" + i, new byte[] {});
      }
      connected = table.get("Connected", false);
    }
  }

  public default void updateInputs(VisionIOInputs inputs) {}
}
