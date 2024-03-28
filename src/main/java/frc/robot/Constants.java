package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Path;
import java.util.function.BooleanSupplier;

public final class Constants {
  public static final Mode kCurrentMode = Mode.kReal;
  public static final RobotName kRobot = RobotName.kBlackMamba;
  public static final boolean kIsViper = kRobot.equals(RobotName.kViper);
  public static final boolean kIsSim = Constants.kCurrentMode.equals(Mode.kSim);
  public static final String kDrivetrainCanBus = "CANivore1";
  public static final String kSuperstructureCanBus = "CANivore2";
  public static final boolean kInTuningMode = true;
  public static final Translation3d poseOffset =
      new Translation3d(Units.inchesToMeters(0), 0.0, 0.0);
  public static final BooleanSupplier onRedAllianceSupplier =
      () ->
          DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red;
  private static final boolean kUseWpiFieldLayout = false;
  public static final AprilTagFieldLayout fieldLayout;

  static {
    if (kUseWpiFieldLayout) {
      try {
        fieldLayout =
            new AprilTagFieldLayout(
                Path.of(
                    Filesystem.getDeployDirectory().getPath(),
                    "apriltags",
                    "2024-wpi-custom.json"));
      } catch (IOException e) {
        throw new RuntimeException(e);
      }
    } else fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
  }

  public static enum Mode {
    kReal,
    kSim,
    kReplay
  }

  public static enum RobotName {
    kViper,
    kBlackMamba
  }
}
