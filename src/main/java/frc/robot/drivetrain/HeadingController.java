package frc.robot.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.PoseEstimation;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class HeadingController {

  private static final double kToleranceDegrees = 1.0;
  private static final double kP = 7.0;
  private static final double kMaxVelocityMultipler = 0.8;
  private static final double kMaxAccelerationMultipler = 0.8;

  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), 0.02);
  ;
  private final Supplier<Rotation2d> m_goalHeadingSupplier;

  public HeadingController(Supplier<Rotation2d> goalHeadingSupplier) {
    m_controller.setP(kP);
    final double maxAngularAcceleration =
        9.0 / Drivetrain.kDriveBaseRadius * kMaxAccelerationMultipler;
    final double maxAngularVelocity = 3.5 / Drivetrain.kDriveBaseRadius * kMaxVelocityMultipler;
    m_controller.setConstraints(
        new TrapezoidProfile.Constraints(maxAngularVelocity, maxAngularAcceleration));
    m_controller.enableContinuousInput(-Math.PI, Math.PI);
    m_controller.setTolerance(Units.degreesToRadians(kToleranceDegrees));
    m_goalHeadingSupplier = goalHeadingSupplier;
    m_controller.reset(
        PoseEstimation.getInstance().getPose().getRotation().getRadians(),
        PoseEstimation.getInstance().getFieldVelocity().dtheta);
  }

  public double update() {
    final double output =
        m_controller.calculate(
            PoseEstimation.getInstance().getPose().getRotation().getRadians(),
            m_goalHeadingSupplier.get().getRadians());
    Logger.recordOutput(
        "HeadingController/Setpoint",
        new Pose2d(
            PoseEstimation.getInstance().getPose().getTranslation(), m_goalHeadingSupplier.get()));
    Logger.recordOutput(
        "HeadingController/HeadingErrorDegrees",
        Units.radiansToDegrees(m_controller.getPositionError()));
    return output;
  }

  @AutoLogOutput(key = "HeadingController/AtGoal")
  public boolean atGoal() {
    return MathUtil.isNear(
        m_controller.getGoal().position,
        m_controller.getSetpoint().position,
        Units.degreesToRadians(kToleranceDegrees));
  }
}
