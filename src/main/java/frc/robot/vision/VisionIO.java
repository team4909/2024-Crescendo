package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public class VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public Pose2d[] poses;
    public int fps;
  }
}
