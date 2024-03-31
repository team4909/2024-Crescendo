package frc.robot;

import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Set;
import java.util.function.Supplier;
import java.util.stream.Stream;
import org.littletonrobotics.junction.Logger;

public class NoteVisualizer {
  private static final double kShotSpeedMps = Units.feetToMeters(35.0);
  private static Supplier<Pose3d> m_wristPoseSupplier;
  private static boolean m_hasNote = false;
  private static final List<Translation2d> m_notes = new ArrayList<>();

  public static void showStagedNotes() {
    if (m_notes.isEmpty()) {
      Logger.recordOutput("NoteVisualizer/StagedNotes", new Pose3d[] {});
    }
    // Show auto notes
    Stream<Translation2d> presentNotes = m_notes.stream().filter(Objects::nonNull);
    Logger.recordOutput(
        "NoteVisualizer/StagedNotes",
        presentNotes
            .map(
                translation ->
                    new Pose3d(
                        translation.getX(),
                        translation.getY(),
                        Units.inchesToMeters(1.0),
                        new Rotation3d()))
            .toArray(Pose3d[]::new));
  }

  public static void showHeldNotes() {
    if (m_hasNote) {
      Logger.recordOutput("NoteVisualizer/HeldNote", getNotePose());
    } else {
      Logger.recordOutput("NoteVisualizer/HeldNote", new Pose3d[] {});
    }
  }

  public static void setHasNote(boolean hasNote) {
    m_hasNote = hasNote;
  }

  public static void removeNote(int note) {
    m_notes.set(note, null);
    m_hasNote = true;
  }

  public static void resetNotes() {
    m_notes.clear();
    for (Translation2d note : FieldConstants.NotePositions.noteTranslations) m_notes.add(note);
  }

  public static Command shoot() {
    return new ScheduleCommand(
        Commands.defer(
                () -> {
                  if (!m_hasNote) return Commands.none();
                  m_hasNote = false;
                  final Pose3d startPose = getNotePose();
                  final Translation3d speakerTranslation =
                      FieldConstants.Speaker.centerSpeakerOpening;
                  final Pose3d endPose;
                  if (Constants.onRedAllianceSupplier.getAsBoolean()) {
                    double flippedX =
                        GeometryUtil.flipFieldPosition(speakerTranslation.toTranslation2d()).getX();
                    endPose =
                        new Pose3d(
                            new Translation3d(
                                flippedX, speakerTranslation.getY(), speakerTranslation.getZ()),
                            startPose.getRotation());
                  } else {
                    endPose = new Pose3d(speakerTranslation, startPose.getRotation());
                  }
                  final double duration =
                      startPose.getTranslation().getDistance(endPose.getTranslation())
                          / kShotSpeedMps;
                  final Timer timer = new Timer();
                  timer.start();
                  return Commands.run(
                          () ->
                              Logger.recordOutput(
                                  "NoteVisualizer/ShotNotes",
                                  new Pose3d[] {
                                    startPose.interpolate(endPose, timer.get() / duration)
                                  }))
                      .until(() -> timer.hasElapsed(duration))
                      .finallyDo(
                          () -> Logger.recordOutput("NoteVisualizer/ShotNotes", new Pose3d[] {}));
                },
                Set.of())
            .withName("Note Shot Visualization")
            .ignoringDisable(true));
  }

  public static void setWristPoseSupplier(Supplier<Pose3d> wristPoseSupplier) {
    m_wristPoseSupplier = wristPoseSupplier;
  }

  private static Pose3d getNotePose() {
    Pose3d wristPose = m_wristPoseSupplier.get();
    Transform3d noteTransform =
        new Transform3d(
                wristPose.getX(), wristPose.getY(), wristPose.getZ(), wristPose.getRotation())
            .plus(new Transform3d(0.1, 0.0, 0.0, new Rotation3d()));
    return new Pose3d(PoseEstimation.getInstance().getPose()).transformBy(noteTransform);
  }
}
