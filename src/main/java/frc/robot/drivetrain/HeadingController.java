package frc.robot.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.LoggedTunableNumber;
import frc.robot.PoseEstimation;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class HeadingController {

  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("HeadingController/kP", 5.0);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("HeadingController/kD", 0.0);
  private static final LoggedTunableNumber maxVelocityMultipler =
      new LoggedTunableNumber("HeadingController/MaxVelocityMultipler", 0.8);
  private static final LoggedTunableNumber maxAccelerationMultipler =
      new LoggedTunableNumber("HeadingController/MaxAccelerationMultipler", 0.8);
  private static final LoggedTunableNumber toleranceDegrees =
      new LoggedTunableNumber("HeadingController/ToleranceDegrees", 1.0);

  private final ProfiledPIDController m_controller;
  private final Supplier<Rotation2d> m_goalHeadingSupplier;

  public HeadingController(Supplier<Rotation2d> goalHeadingSupplier) {
    m_controller =
        new ProfiledPIDController(
            kP.get(), 0, kD.get(), new TrapezoidProfile.Constraints(0.0, 0.0), 0.02);
    m_controller.enableContinuousInput(-Math.PI, Math.PI);
    m_controller.setTolerance(Units.degreesToRadians(toleranceDegrees.get()));
    m_goalHeadingSupplier = goalHeadingSupplier;

    m_controller.reset(
        PoseEstimation.getInstance().getPose().getRotation().getRadians(),
        PoseEstimation.getInstance().getFieldVelocity().dtheta);
  }

  public double update() {
    m_controller.setPID(kP.get(), 0, kD.get());
    m_controller.setTolerance(Units.degreesToRadians(toleranceDegrees.get()));

    final double maxAngularAcceleration =
        9.0 / Drivetrain.kDriveBaseRadius * maxAccelerationMultipler.get();
    final double maxAngularVelocity =
        3.5 / Drivetrain.kDriveBaseRadius * maxVelocityMultipler.get();
    m_controller.setConstraints(
        new TrapezoidProfile.Constraints(maxAngularVelocity, maxAngularAcceleration));

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
        Units.degreesToRadians(toleranceDegrees.get()));
  }
}
