package frc.robot;

import com.choreo.lib.Choreo;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.feeder.Feeder;
import frc.robot.intake.Intake;
import frc.robot.shooter.Shooter;

public class Autos {

  private final Drivetrain m_drivetrain;
  private final Shooter m_shooter;
  private final Feeder m_feeder;
  private final Intake m_intake;

  public Autos(Drivetrain drivetrain, Shooter shooter, Feeder feeder, Intake intake) {
    m_drivetrain = drivetrain;
    m_shooter = shooter;
    m_feeder = feeder;
    m_intake = intake;
  }

  public Command centerlineTwoPiece() {
    return Commands.sequence(
            resetPose("Centerline Auto"),
            shoot(),
            intake().deadlineWith(getPathFollowingCommand("Centerline Auto.1")),
            getPathFollowingCommand("Centerline Auto.2"),
            shoot(),
            intake().deadlineWith(getPathFollowingCommand("Centerline Auto.3")),
            getPathFollowingCommand("Centerline Auto.4"),
            shoot())
        .withName("Centerline 3 Piece");
  }

  private Command getPathFollowingCommand(String trajectoryName) {
    return AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(trajectoryName));
  }

  private Command resetPose(String trajectoryName) {
    Pose2d startingPose = Choreo.getTrajectory(trajectoryName).getInitialPose();
    return Commands.runOnce(
        () ->
            m_drivetrain.resetPose.accept(
                m_drivetrain.onRedAllianceSupplier.getAsBoolean()
                    ? GeometryUtil.flipFieldPose(startingPose)
                    : startingPose));
  }

  private Command intake() {
    return Superstructure.sensorIntake(m_feeder, m_intake)
        .andThen(m_intake.stop(), m_feeder.stop());
  }

  private Command shoot() {
    return Commands.race(
            m_shooter.runShooter(),
            Commands.waitUntil(m_shooter.readyToShoot()).andThen(m_feeder.shoot().withTimeout(0.5)))
        .andThen(m_shooter.stop(), m_feeder.stop())
        .finallyDo(() -> System.out.println("Shooting complete!"));
  }
}
