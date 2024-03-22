package frc.robot.vision;

import frc.robot.Constants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class GamePieceDetection {
  private final double kTargetAreaThreshold = 0.0;
  private final GamePieceDetectionIO m_io;
  private final GamePieceDetectionIOInputsAutoLogged m_inputs =
      new GamePieceDetectionIOInputsAutoLogged();

  public final BooleanSupplier hasValidTarget =
      () -> m_inputs.hasTarget || m_inputs.targetArea > kTargetAreaThreshold;
  public final DoubleSupplier horizontalError =
      () ->
          m_inputs.targetHorizontalOffsetDegrees
              * (Constants.onRedAllianceSupplier.getAsBoolean() ? -1 : 1);

  public GamePieceDetection(GamePieceDetectionIO io) {
    m_io = io;
  }

  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("VisionInputs/Limelight", m_inputs);
    Logger.recordOutput("GamePieceDetection/ValidTarget", hasValidTarget.getAsBoolean());
    Logger.recordOutput("GamePieceDetection/HorizontalError", horizontalError.getAsDouble());
  }
}
