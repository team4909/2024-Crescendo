package frc.robot.arm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

public class ArmVisualizer {

  // Value from CAD, angle from horizontal to shoulder.
  private final double kShoulderAngleDegrees = 103.985388;
  private final Translation2d origin =
      new Translation2d(Units.inchesToMeters(-9.2), Units.inchesToMeters(20));
  private final String m_logKey;
  private final Mechanism2d m_mechanism;
  private final MechanismRoot2d m_mechanismRoot;
  private final MechanismLigament2d m_fixedShoulderLigament, m_elbowLigament, m_wristLigament;
  private Pose3d m_elbowPose, m_wristPose;

  public ArmVisualizer(String logKey, double ligamentWidth, Color color) {
    m_logKey = logKey;
    m_mechanism = new Mechanism2d(4, 3, new Color8Bit(Color.kGray));
    m_mechanismRoot = m_mechanism.getRoot("Arm", 2 + origin.getX(), 0);
    m_fixedShoulderLigament =
        m_mechanismRoot.append(
            new MechanismLigament2d(
                "Shoulder", origin.getY(), kShoulderAngleDegrees, 6, new Color8Bit(Color.kBlack)));
    m_elbowLigament =
        m_fixedShoulderLigament.append(
            new MechanismLigament2d(
                "Elbow", ArmConstants.kElbowLengthMeters, 0, ligamentWidth, new Color8Bit(color)));
    m_wristLigament =
        m_elbowLigament.append(
            new MechanismLigament2d(
                "Wrist",
                ArmConstants.kWristLengthMeters,
                0,
                ligamentWidth,
                new Color8Bit(color)));
  }

  public void update(double elbowAngleRad, double wristAngleRad) {
    m_elbowLigament.setAngle(Units.radiansToDegrees(elbowAngleRad) - kShoulderAngleDegrees);
    m_wristLigament.setAngle(Units.radiansToDegrees(wristAngleRad - elbowAngleRad));
    Logger.recordOutput("Mechanism2d/" + m_logKey, m_mechanism);

    m_elbowPose =
        new Pose3d(origin.getX(), 0.0, origin.getY(), new Rotation3d(0.0, -elbowAngleRad, 0.0));
    m_wristPose =
        m_elbowPose.transformBy(
            new Transform3d(
                new Translation3d(ArmConstants.kElbowLengthMeters, 0.0, 0.0),
                new Rotation3d(0.0, -(wristAngleRad - elbowAngleRad), 0.0)));
    Logger.recordOutput("Mechanism3d/" + m_logKey, m_elbowPose, m_wristPose);
  }

  public Pose3d getWristPose() {
    return m_wristPose;
  }
}
