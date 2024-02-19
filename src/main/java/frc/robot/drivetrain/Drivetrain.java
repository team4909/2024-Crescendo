package frc.robot.drivetrain;

import static edu.wpi.first.units.Units.Volts;

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
import edu.wpi.first.math.geometry.Twist2d;
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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.LocalADStarAK;
import frc.robot.Constants;
import frc.robot.vision.Vision.VisionUpdate;
import java.util.Arrays;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.UnaryOperator;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drivetrain extends SubsystemBase {

  public static final Lock odometryLock = new ReentrantLock();
  public final BooleanSupplier onRedAllianceSupplier;

  public final double kTrackwidthMeters = Units.inchesToMeters(26.0);
  public final double kWheelbaseMeters = Units.inchesToMeters(26.0);
  private final double kDriveBaseRadius =
      Math.hypot(kTrackwidthMeters / 2.0, kWheelbaseMeters / 2.0);
  private final double kMaxLinearSpeedMetersPerSecond = Units.feetToMeters(16.5);
  private final double kMaxAngularSpeedRadPerSec = 2 * Math.PI;
  private final double kDeadband = 0.07;
  private final boolean kUseVisionCorrection = true;
  private final ImuIO m_imuIO;
  private final ImuIOInputsAutoLogged m_imuInputs = new ImuIOInputsAutoLogged();
  private final Module[] m_modules = new Module[4]; // FL, FR, BL, BR
  private final SwerveDrivePoseEstimator m_poseEstimator;
  private final Consumer<VisionUpdate> m_visionUpdateConsumer;
  private final SysIdRoutine m_sysIdRoutine;
  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          new Translation2d(kTrackwidthMeters / 2.0, kWheelbaseMeters / 2.0),
          new Translation2d(kTrackwidthMeters / 2.0, -kWheelbaseMeters / 2.0),
          new Translation2d(-kTrackwidthMeters / 2.0, kWheelbaseMeters / 2.0),
          new Translation2d(-kTrackwidthMeters / 2.0, -kWheelbaseMeters / 2.0));
  public final Pose2d m_sourcePoseBlueOrigin =
      new Pose2d(15.40, 0.95, Rotation2d.fromDegrees(-60.0));

  private Twist2d m_fieldVelocity = new Twist2d();

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
    onRedAllianceSupplier =
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;
    m_sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drivetrain/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                  for (Module module : m_modules) {
                    module.runCharacterization(voltage.in(Volts));
                  }
                },
                null,
                this));
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
    Logger.processInputs("DrivetrainInputs/IMU", m_imuInputs);
    for (Module module : m_modules) {
      module.periodic();
    }

    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
      for (Module module : m_modules) {
        module.stop();
      }
    }

    ChassisSpeeds chassisSpeeds = m_kinematics.toChassisSpeeds(getModuleStates());
    Translation2d linearFieldVelocity =
        new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
            .rotateBy(getPose().getRotation());
    m_fieldVelocity =
        new Twist2d(
            linearFieldVelocity.getX(),
            linearFieldVelocity.getY(),
            m_imuInputs.yawVelocityRadPerSec);

    double[] sampleTimestamps = m_modules[0].getOdometryTimestamps();
    for (int updateIndex = 0; updateIndex < sampleTimestamps.length; updateIndex++) {
      SwerveModulePosition[] newModulePositions = new SwerveModulePosition[m_modules.length];
      for (int moduleIndex = 0; moduleIndex < m_modules.length; moduleIndex++) {
        newModulePositions[moduleIndex] =
            m_modules[moduleIndex].getOdometryPositions()[updateIndex];
      }

      // The reason we are bothering with timestamps here is so vision updates can be properly
      // chronologized with odometry updates.
      m_poseEstimator.updateWithTime(
          sampleTimestamps[updateIndex],
          m_imuInputs.odometryYawPositions[updateIndex],
          newModulePositions);
    }
  }

  public void runVelocity(ChassisSpeeds speeds) {
    if (Constants.kIsSim) {
      m_imuIO.updateSim(speeds.omegaRadiansPerSecond * 0.02);
    }
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

  public Command joystickDrive(
      DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
    UnaryOperator<Double> squareAxis =
        (Double axisMagnitude) -> Math.copySign(Math.pow(axisMagnitude, 2), axisMagnitude);
    return this.run(
            () -> {
              var x = squareAxis.apply(MathUtil.applyDeadband(xSupplier.getAsDouble(), kDeadband));
              var y = squareAxis.apply(MathUtil.applyDeadband(ySupplier.getAsDouble(), kDeadband));
              double omega =
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

  public Command zeroGyro() {
    return Commands.runOnce(() -> m_imuIO.setGyroAngle(0.0)).ignoringDisable(true);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
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
        onRedAllianceSupplier,
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

  public Twist2d getFieldVelocity() {
    return m_fieldVelocity;
  }

  public double getYawVelocity() {
    return m_imuInputs.yawVelocityRadPerSec;
  }

  private void setPose(Pose2d pose) {
    m_poseEstimator.resetPosition(m_imuInputs.yawPosition, getModulePositions(), pose);
  }

  public Consumer<VisionUpdate> getVisionPoseConsumer() {
    return m_visionUpdateConsumer;
  }
}
