package frc.robot.arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

public class ArmVisualizer {

  private final double kFixedLigamentAngle = 76.015;
  private final String m_logKey;
  private final Mechanism2d m_mechanism;
  private final MechanismRoot2d m_mechanismRoot;
  private final MechanismLigament2d m_fixedLigament;
  private final MechanismLigament2d m_elbowLigament;
  private final MechanismLigament2d m_wristLigament;

  public ArmVisualizer(String logKey) {
    m_logKey = logKey;
    m_mechanism = new Mechanism2d(4, 3, new Color8Bit(Color.kGray));
    m_mechanismRoot = m_mechanism.getRoot("Arm", 2 + ArmConfig.kOrigin.getX(), 0);
    m_fixedLigament =
        m_mechanismRoot.append(
            new MechanismLigament2d(
                "Fixed", ArmConfig.kOrigin.getY(), 90.0, 6, new Color8Bit(Color.kBlack)));
    m_elbowLigament =
        m_fixedLigament.append(
            new MechanismLigament2d(
                "Elbow", ArmConfig.kElbowConfig.length(), 0, 4, new Color8Bit(Color.kDarkBlue)));
    m_wristLigament =
        m_elbowLigament.append(
            new MechanismLigament2d(
                "Wrist", ArmConfig.kWristConfig.length(), 0, 4, new Color8Bit(Color.kBlue)));
  }

  public void update(double elbowAngle, double wristAngle) {
    m_elbowLigament.setAngle(Units.radiansToDegrees(elbowAngle));
    m_wristLigament.setAngle(Units.radiansToDegrees(wristAngle));
    Logger.recordOutput("Mechanism2d/" + m_logKey, m_mechanism);
  }
}
