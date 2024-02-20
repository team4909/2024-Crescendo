package frc.robot;

public final class Constants {
  public static final Mode kCurrentMode = Mode.kReal;
  public static final RobotType kRobot = RobotType.kPractice;
  public static final boolean kIsSim = Constants.kCurrentMode.equals(Mode.kSim);
  public static final String kDrivetrainCanBus = "CANivore1";
  public static final String kOtherCanBus = "CANivore2";
  public static final boolean kInTuningMode = true;

  public static enum Mode {
    kReal,
    kSim,
    kReplay
  }

  public static enum RobotType {
    kCompetition,
    kPractice
  }
}
