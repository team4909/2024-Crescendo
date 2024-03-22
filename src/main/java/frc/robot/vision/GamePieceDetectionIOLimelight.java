package frc.robot.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class GamePieceDetectionIOLimelight implements GamePieceDetectionIO {

  private final NetworkTable m_limelightTable;
  private final double kDisconnectedTimeoutSeconds = 0.5;
  private final Timer m_disconnectedTimer = new Timer();
  private double m_lastHeartbeatValue = 0.0;

  public GamePieceDetectionIOLimelight() {
    m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    m_limelightTable.getEntry("pipeline").setNumber(0);
  }

  @Override
  public void updateInputs(GamePieceDetectionIOInputs inputs) {
    inputs.hasTarget = m_limelightTable.getEntry("tv").getDouble(0) == 1.0;
    inputs.targetArea = m_limelightTable.getEntry("ta").getDouble(0.0);
    inputs.targetHorizontalOffsetDegrees = m_limelightTable.getEntry("tx").getDouble(0.0);
    inputs.targetVerticalOffsetDegrees = m_limelightTable.getEntry("ty").getDouble(0.0);
    inputs.fps = (int) m_limelightTable.getEntry("hw").getDoubleArray(new double[] {0.0})[0];

    double currentHeartbeatValue = m_limelightTable.getEntry("hb").getDouble(0.0);
    if (m_lastHeartbeatValue != currentHeartbeatValue) {
      m_lastHeartbeatValue = currentHeartbeatValue;
      inputs.connected = true;
      m_disconnectedTimer.reset();
    } else if (m_disconnectedTimer.hasElapsed(kDisconnectedTimeoutSeconds)) {
      inputs.connected = false;
    }
  }
}
