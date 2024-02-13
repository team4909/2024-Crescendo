package frc.robot.arm;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public enum ArmSetpoints {
  kStowed(-5, 13, 0.8, 0.0),
  kTrap(19.8, 30, 0.0, 0.6);

  private final Translation2d m_setpoint;
  private final double m_elbowDelay;
  private final double m_wristDelay;

  private ArmSetpoints(Translation2d setpoint, double elbowDelay, double wristDelay) {
    m_setpoint = setpoint;
    m_elbowDelay = elbowDelay;
    m_wristDelay = wristDelay;
  }

  private ArmSetpoints(
      double xPositionInches, double yPositionInches, double elbowDelay, double wristDelay) {
    this(
        new Translation2d(
            Units.inchesToMeters(xPositionInches), Units.inchesToMeters(yPositionInches)),
        elbowDelay,
        wristDelay);
  }

  public Translation2d getTranslation() {
    return m_setpoint;
  }

  public Pair<Double, Double> getDelay() {
    return Pair.of(m_elbowDelay, m_wristDelay);
  }
}
