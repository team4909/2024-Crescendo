package frc.robot.drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.LocalADStarAK;
import frc.robot.Constants;
import frc.robot.vision.Vision.VisionUpdate;
import java.util.Arrays;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.UnaryOperator;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drivetrain extends SubsystemBase {

  public final double kTrackwidthMeters = Units.inchesToMeters(26.0);
  public final double kWheelbaseMeters = Units.inchesToMeters(26.0);
  private final double kDriveBaseRadius =
      Math.hypot(kTrackwidthMeters / 2.0, kWheelbaseMeters / 2.0);
  private final double kMaxLinearSpeedMetersPerSecond = Units.feetToMeters(16.5);
  private final double kMaxAngularSpeedRadPerSec = 2 * Math.PI;
  private final double kDeadband = 0.05;
  private final boolean kUseVisionCorrection = false;

  public static final Lock odometryLock = new ReentrantLock();
  private final ImuIO m_imuIO;
  private final ImuIOInputsAutoLogged m_imuInputs = new ImuIOInputsAutoLogged();
  private final Module[] m_modules = new Module[4]; // FL, FR, BL, BR
  private final SwerveDrivePoseEstimator m_poseEstimator;
  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          new Translation2d(kTrackwidthMeters / 2.0, kWheelbaseMeters / 2.0),
          new Translation2d(kTrackwidthMeters / 2.0, -kWheelbaseMeters / 2.0),
          new Translation2d(-kTrackwidthMeters / 2.0, kWheelbaseMeters / 2.0),
          new Translation2d(-kTrackwidthMeters / 2.0, -kWheelbaseMeters / 2.0));

  // For calculating chassis position deltas in simulation.
  private SwerveModulePosition[] m_lastModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  private final Consumer<VisionUpdate> m_visionUpdateConsumer;

  public Drivetrain(
      ImuIO imuIO,
      ModuleIO frontLeftModuleIO,
      ModuleIO frontRightModuleIO,
      ModuleIO backLeftModuleIO,
      ModuleIO backRightModuleIO) {
    this.m_imuIO = imuIO;
    m_modules[0] = new Module(frontLeftModuleIO, 0);
    m_modules[1] = new Module(frontRightModuleIO, 1);
    m_modules[2] = new Module(backLeftModuleIO, 2);
    m_modules[3] = new Module(backRightModuleIO, 3);
    PhoenixOdometryThread.getInstance().start();
    m_poseEstimator =
        new SwerveDrivePoseEstimator(
            m_kinematics, new Rotation2d(), getModulePositions(), new Pose2d());
    m_visionUpdateConsumer =
        (VisionUpdate visionUpdate) -> {
          if (!kUseVisionCorrection) {
            return;
          }
          m_poseEstimator.addVisionMeasurement(
              visionUpdate.pose(),
              visionUpdate.timestampSeconds(),
              visionUpdate.standardDeviations());
        };
    configurePathing();
  }

  public void periodic() {

    // Make the thread stop polling signals while updating odometry readings.
    odometryLock.lock();
    m_imuIO.updateInputs(m_imuInputs);
    for (Module module : m_modules) {
      module.updateInputs();
    }

    odometryLock.unlock();
    Logger.processInputs("Drive/IMU", m_imuInputs);
    for (var module : m_modules) {
      module.periodic();
    }

    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
      for (Module module : m_modules) {
        module.stop();
      }
    }

    double[] sampleTimestamps = m_modules[0].getOdometryTimestamps();
    for (int updateIndex = 0; updateIndex < sampleTimestamps.length; updateIndex++) {
      SwerveModulePosition[] newModulePositions = new SwerveModulePosition[m_modules.length];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < m_modules.length; moduleIndex++) {
        newModulePositions[moduleIndex] =
            m_modules[moduleIndex].getOdometryPositions()[updateIndex];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                newModulePositions[moduleIndex].distanceMeters
                    - m_lastModulePositions[moduleIndex].distanceMeters,
                newModulePositions[moduleIndex].angle);
        m_lastModulePositions[moduleIndex] = newModulePositions[moduleIndex];
      }
      if (Constants.kIsSim) {
        m_imuIO.updateSim(m_kinematics.toTwist2d(moduleDeltas).dtheta);
      }

      // The reason we are bothering with timestamps here is so vision updates can be properly
      // chronologized with odometry updates.
      m_poseEstimator.updateWithTime(
          sampleTimestamps[updateIndex],
          m_imuInputs.odometryYawPositions[updateIndex],
          newModulePositions);
    }
  }

  private void runVelocity(ChassisSpeeds speeds) {
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = m_kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, kMaxLinearSpeedMetersPerSecond);

    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int moduleIndex = 0; moduleIndex < m_modules.length; moduleIndex++) {
      optimizedSetpointStates[moduleIndex] =
          m_modules[moduleIndex].setSetpoint(setpointStates[moduleIndex]);
    }

    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  public Command testDrive() {
    return this.run(
            () -> {
              runVelocity(new ChassisSpeeds(1, 1, 0));
            })
        .withName("Test Drive");
  }

  public Command joystickDrive(
      DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
    UnaryOperator<Double> squareAxis =
        (Double axisMagnitude) -> Math.copySign(Math.pow(axisMagnitude, 2), axisMagnitude);
    return this.run(
            () -> {
              var x = squareAxis.apply(MathUtil.applyDeadband(xSupplier.getAsDouble(), kDeadband));
              var y = squareAxis.apply(MathUtil.applyDeadband(ySupplier.getAsDouble(), kDeadband));
              var omega =
                  squareAxis.apply(MathUtil.applyDeadband(omegaSupplier.getAsDouble(), kDeadband));
              runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      x * kMaxLinearSpeedMetersPerSecond,
                      y * kMaxLinearSpeedMetersPerSecond,
                      omega * kMaxAngularSpeedRadPerSec,
                      m_imuInputs.yawPosition));
            })
        .withName("Joystick Drive");
  }

  public Command zeroRotation() {
    return Commands.runOnce(() -> setPose(new Pose2d(getPose().getTranslation(), new Rotation2d())))
        .ignoringDisable(true);
  }

  private void configurePathing() {
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::setPose,
        () -> m_kinematics.toChassisSpeeds(getModuleStates()),
        this::runVelocity,
        new HolonomicPathFollowerConfig(
            new PIDConstants(5.0, 0.0, 0.0),
            new PIDConstants(5.0, 0.0, 0.0),
            kMaxLinearSpeedMetersPerSecond,
            kDriveBaseRadius,
            new ReplanningConfig()),
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Drivetrain/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Drivetrain/TrajectorySetpoint", targetPose);
        });
  }

  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    return Arrays.stream(m_modules).map(Module::getState).toArray(SwerveModuleState[]::new);
  }

  private SwerveModulePosition[] getModulePositions() {
    return Arrays.stream(m_modules).map(Module::getPosition).toArray(SwerveModulePosition[]::new);
  }

  @AutoLogOutput(key = "Drivetrain/Estimated Pose")
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void setPose(Pose2d pose) {
    m_poseEstimator.resetPosition(m_imuInputs.yawPosition, getModulePositions(), pose);
  }

  public Consumer<VisionUpdate> getVisionPoseConsumer() {
    return m_visionUpdateConsumer;
  }
}
