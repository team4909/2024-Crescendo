package frc.robot.vision;

import org.littletonrobotics.junction.AutoLog;

public interface GamePieceDetectionIO {

  @AutoLog
  public static class GamePieceDetectionIOInputs {
    public boolean hasTarget = false;
    public double targetArea = 0.0;
    public double targetHorizontalOffsetDegrees = 0.0;
    public double targetVerticalOffsetDegrees = 0.0;
    public int fps;
    public boolean connected = false;
  }

  public default void updateInputs(GamePieceDetectionIOInputs inputs) {}
}
