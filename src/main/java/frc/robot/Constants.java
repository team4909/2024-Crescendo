package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.function.BooleanSupplier;

public final class Constants {
  public static final Mode kCurrentMode = Mode.kSim;
  public static final RobotName kRobot = RobotName.kViper;
  public static final boolean kIsSim = Constants.kCurrentMode.equals(Mode.kSim);
  public static final String kDrivetrainCanBus = "CANivore1";
  public static final String kSuperstructureCanBus = "CANivore2";
  public static final boolean kInTuningMode = true;
  public static final BooleanSupplier onRedAllianceSupplier =
      () ->
          DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red;

  public static enum Mode {
    kReal,
    kSim,
    kReplay
  }

  public static enum RobotName {
    kViper,
    kCobra
  }
}
