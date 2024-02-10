package frc.robot;

public final class Constants {
  public static final Mode kCurrentMode = Mode.kSim;
  public static final RobotType kRobot = RobotType.kPractice;
  public static final boolean kIsSim = Constants.kCurrentMode.equals(Mode.kSim);
  public static final String kCanBusName = "CANivore1";
  public static final boolean kInTuningMode = false;

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
