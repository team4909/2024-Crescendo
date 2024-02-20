package frc.robot.arm;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public enum ArmSetpoints {
  kStowed(new Translation2d(-0.102, 0.515), 0.0, 0.0),
  kSubwoofer(new Translation2d(-0.053, -0.015), 0.0, 0.0),
  kSourceCatch(new Translation2d(), 0.0, 0.0),
  kClimbPreparation(new Translation2d(), 0.0, 0.0);

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
