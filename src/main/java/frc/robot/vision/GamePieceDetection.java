package frc.robot.vision;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class GamePieceDetection {
  private final double kMinimumTargetAreaPercent = 0.8;
  private final GamePieceDetectionIO m_io;
  private final GamePieceDetectionIOInputsAutoLogged m_inputs =
      new GamePieceDetectionIOInputsAutoLogged();

  public final Trigger hasValidTarget =
      new Trigger(
          () ->
              m_inputs.connected
                  && m_inputs.hasTarget
                  && m_inputs.targetArea > kMinimumTargetAreaPercent);
  public final DoubleSupplier horizontalErrorDeg =
      () ->
          m_inputs.targetHorizontalOffsetDegrees
              * (Constants.onRedAllianceSupplier.getAsBoolean() ? 1 : -1);

  public GamePieceDetection(GamePieceDetectionIO io) {
    m_io = io;
  }

  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("VisionInputs/Limelight", m_inputs);
    Logger.recordOutput("GamePieceDetection/ValidTarget", hasValidTarget.getAsBoolean());
    Logger.recordOutput("GamePieceDetection/HorizontalErrorDeg", horizontalErrorDeg.getAsDouble());
  }
}
