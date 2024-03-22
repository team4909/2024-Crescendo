package frc.robot.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.FieldConstants.NotePositions;
import frc.robot.NoteVisualizer;
import frc.robot.PoseEstimation;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO m_io;
  private final IntakeIOInputsAutoLogged m_inputs = new IntakeIOInputsAutoLogged();
  private boolean m_isIntakingPieceSim = false;
  private final double kIntakeTimeSimSeconds = 0.1;
  private final List<Integer> m_intookPieces = new ArrayList<>();
  public final Trigger hasIntookPieceSim =
      new Trigger(() -> m_isIntakingPieceSim).debounce(kIntakeTimeSimSeconds, DebounceType.kRising);

  public Intake(IntakeIO io) {
    m_io = io;
    setDefaultCommand(idle());
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("IntakeInputs", m_inputs);
    Logger.recordOutput("Intake/Sim/HasIntookPiece", hasIntookPieceSim.getAsBoolean());
  }

  public Command idle() {
    return this.run(
        () -> {
          m_io.setTopRollerDutyCycle(0.0);
          m_io.setBottomRollerDutyCycle(0.0);
          m_io.setCenteringMotorsDutyCycle(0.0);
        });
  }

  public Command stop() {
    return this.runOnce(m_io::stopRollers);
  }

  public Command spit() {
    return this.run(
        () -> {
          m_io.setTopRollerDutyCycle(-0.8);
          m_io.setBottomRollerDutyCycle(-0.8);
          m_io.setCenteringMotorsDutyCycle(-0.5);
        });
  }

  public Command intake() {
    return this.run(
            () -> {
              m_io.setTopRollerDutyCycle(0.8);
              m_io.setBottomRollerDutyCycle(0.8);
              m_io.setCenteringMotorsDutyCycle(0.85);
            })
        .alongWith(
            Commands.run(() -> m_isIntakingPieceSim = simulateIsIntakingPiece())
                .until(() -> m_isIntakingPieceSim)
                .andThen(Commands.run(() -> m_isIntakingPieceSim = true))
                .finallyDo(() -> m_isIntakingPieceSim = false)
                .unless(() -> !Constants.kIsSim))
        .withName("Intake");
  }

  private boolean simulateIsIntakingPiece() {
    final Translation2d intakeOffsetFromRobotCenter =
        new Translation2d(Units.inchesToMeters(10.0), 0.0);
    final Translation2d intakePosition =
        PoseEstimation.getInstance().getPose().getTranslation().plus(intakeOffsetFromRobotCenter);
    final double kIntakeXRangeInches = 9.0;
    final double kIntakeYRangeInches = 27.0 / 2.0;
    for (int noteIndex = 0; noteIndex < NotePositions.noteTranslations.length; noteIndex++) {
      if (m_intookPieces.contains(noteIndex)) continue;
      final Translation2d intakeToNote =
          NotePositions.noteTranslations[noteIndex].minus(intakePosition);
      if (MathUtil.isNear(
              0, Math.abs(intakeToNote.getX()), Units.inchesToMeters(kIntakeXRangeInches))
          && MathUtil.isNear(
              0, Math.abs(intakeToNote.getY()), Units.inchesToMeters(kIntakeYRangeInches))) {
        m_intookPieces.add(noteIndex);
        NoteVisualizer.removeNote(noteIndex);
        return true;
      }
    }
    return false;
  }

  public Command feed() {
    return this.run(
        () -> {
          m_io.setTopRollerDutyCycle(0.4);
          m_io.setBottomRollerDutyCycle(0.4);
          m_io.setCenteringMotorsDutyCycle(0.5);
        });
  }
}
