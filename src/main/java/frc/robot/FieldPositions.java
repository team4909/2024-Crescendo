package frc.robot;

import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class FieldPositions {
  public static class NotePositions {
    public static final double fieldLength = Constants.fieldLayout.getFieldLength();
    private static final double kCenterlineX = fieldLength / 2.0;
    private static final double kCenterlineFirstY = Units.inchesToMeters(29.638);
    private static final double kCenterlineSeparationY = Units.inchesToMeters(66);
    private static final double kSpikeX = Units.inchesToMeters(114);
    private static final double kSpikeFirstY = Units.inchesToMeters(161.638);
    private static final double kSpikeSeparationY = Units.inchesToMeters(57);
    private static final double kCenterlineNoteCount = 5;
    private static final double kSpikeNoteCount = 3;
    public static final Translation2d[] noteTranslations = new Translation2d[11];

    static {
      int i = 0;
      for (int j = 0; j < kCenterlineNoteCount; j++, i++)
        noteTranslations[i] =
            new Translation2d(kCenterlineX, kCenterlineFirstY + (j * kCenterlineSeparationY));
      for (int j = 0; j < kSpikeNoteCount; j++, i++)
        noteTranslations[i] = new Translation2d(kSpikeX, kSpikeFirstY + (j * kSpikeSeparationY));
      for (int j = 0; j < kSpikeNoteCount; j++, i++)
        noteTranslations[i] =
            GeometryUtil.flipFieldPosition(
                new Translation2d(kSpikeX, kSpikeFirstY + (j * kSpikeSeparationY)));
    }
  }

  public static final class Speaker {
    private static final Translation3d topRightSpeaker =
        new Translation3d(
            Units.inchesToMeters(18.055),
            Units.inchesToMeters(238.815),
            Units.inchesToMeters(83.091));
    private static final Translation3d bottomLeftSpeaker =
        new Translation3d(0.0, Units.inchesToMeters(197.765), Units.inchesToMeters(78.324));
    public static final Translation3d centerSpeakerOpening =
        bottomLeftSpeaker.interpolate(topRightSpeaker, 0.5);
  }
}
