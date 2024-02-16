package frc.robot.arm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
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
  private final String m_logKey;
  private final Mechanism2d m_mechanism;
  private final MechanismRoot2d m_mechanismRoot;
  private final MechanismLigament2d m_fixedShoulderLigament;
  private final MechanismLigament2d m_elbowLigament;
  private final MechanismLigament2d m_wristLigament;

  public ArmVisualizer(String logKey) {
    m_logKey = logKey;
    m_mechanism = new Mechanism2d(4, 3, new Color8Bit(Color.kGray));
    m_mechanismRoot = m_mechanism.getRoot("Arm", 2 + ArmModel.kOrigin.getX(), 0);
    m_fixedShoulderLigament =
        m_mechanismRoot.append(
            new MechanismLigament2d(
                "Shoulder",
                ArmModel.kOrigin.getY(),
                kShoulderAngleDegrees,
                6,
                new Color8Bit(Color.kBlack)));
    m_elbowLigament =
        m_fixedShoulderLigament.append(
            new MechanismLigament2d(
                "Elbow", ArmModel.kElbowLengthMeters, 0, 4, new Color8Bit(Color.kGreen)));
    m_wristLigament =
        m_elbowLigament.append(
            new MechanismLigament2d(
                "Wrist", ArmModel.kWristLengthMeters, 0, 4, new Color8Bit(Color.kDarkGreen)));
  }

  public void update(double elbowAngle, double wristAngle) {
    m_elbowLigament.setAngle(Units.radiansToDegrees(elbowAngle) - kShoulderAngleDegrees);
    m_wristLigament.setAngle(Units.radiansToDegrees(wristAngle - elbowAngle));
    Logger.recordOutput("Mechanism2d/" + m_logKey, m_mechanism);

    Pose3d elbowPose =
        new Pose3d(
            ArmModel.kOrigin.getX(),
            0.0,
            ArmModel.kOrigin.getY(),
            new Rotation3d(0.0, -elbowAngle, 0.0));
    Pose3d wristPose =
        elbowPose.transformBy(
            new Transform3d(
                new Translation3d(ArmModel.kElbowLengthMeters, 0.0, 0.0),
                new Rotation3d(0.0, -(wristAngle - elbowAngle), 0.0)));
    Logger.recordOutput("Mechanism3d/" + m_logKey, elbowPose, wristPose);
  }
}
