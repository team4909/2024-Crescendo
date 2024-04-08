package frc.robot.drivetrain;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.GeometryUtil;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.PoseEstimation;
import java.util.Arrays;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.function.UnaryOperator;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drivetrain extends SubsystemBase {

  public static final Lock odometryLock = new ReentrantLock();

  private static final double kTrackwidthMeters = Units.inchesToMeters(20.75);
  private static double kWheelbaseMeters = Units.inchesToMeters(15.75);
  private static final Translation2d[] m_modulePositions =
      new Translation2d[] {
        new Translation2d(kTrackwidthMeters / 2.0, kWheelbaseMeters / 2.0),
        new Translation2d(kTrackwidthMeters / 2.0, -kWheelbaseMeters / 2.0),
        new Translation2d(-kTrackwidthMeters / 2.0, kWheelbaseMeters / 2.0),
        new Translation2d(-kTrackwidthMeters / 2.0, -kWheelbaseMeters / 2.0)
      };
  public static final SwerveDriveKinematics swerveKinematics =
      new SwerveDriveKinematics(m_modulePositions);
  public static final double kDriveBaseRadius =
      Math.hypot(kTrackwidthMeters / 2.0, kWheelbaseMeters / 2.0);
  private static final LoggedTunableNumber speakerRangeMeters =
      new LoggedTunableNumber("Drivetrain/InRangeRadius", 5.0);
  private final double kMaxLinearSpeedMetersPerSecond = Units.feetToMeters(16);
  private final double kMaxAngularSpeedRadPerSec = 2 * Math.PI;
  private final double kDeadband = 0.1;
  private final ImuIO m_imuIO;
  private final ImuIOInputsAutoLogged m_imuInputs = new ImuIOInputsAutoLogged();
  private final Module[] m_modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine m_sysIdRoutineDrive, m_sysIdRoutineRotation;

  private Rotation2d m_gyroRotation = new Rotation2d();
  private HeadingController m_headingController;
  private SwerveModulePosition[] m_lastModulePositions = {
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition()
  };

  public final Trigger atHeadingGoal = new Trigger(this::atHeadingGoal);
  public final Trigger inRangeOfGoal = new Trigger(this::inRange);
  public final Pose2d m_sourcePoseBlueOrigin =
      new Pose2d(15.40, 0.95, Rotation2d.fromDegrees(-60.0));

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

    m_sysIdRoutineDrive =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> SignalLogger.writeString("Drivetrain/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                  for (Module module : m_modules) {
                    module.runCharacterization(voltage.in(Volts));
                  }
                },
                null,
                this));
    m_sysIdRoutineRotation =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(4.0),
                null,
                (state) -> Logger.recordOutput("Drivetrain/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                  for (int i = 0; i < m_modules.length; ++i) {
                    m_modules[i].runCharacterization(
                        m_modulePositions[i].getAngle().plus(Rotation2d.fromDegrees(90)),
                        voltage.in(Volts));
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
      Logger.recordOutput("Drivetrain/SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("Drivetrain/SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
      for (Module module : m_modules) module.stop();
    }

    final double[] sampleTimestamps = m_modules[0].getOdometryTimestamps();
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

      // Update gyro angle
      if (m_imuInputs.connected) {
        m_gyroRotation = m_imuInputs.odometryYawPositions[updateIndex];
      } else {
        final Twist2d twist = swerveKinematics.toTwist2d(moduleDeltas);
        m_gyroRotation = m_gyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      PoseEstimation.getInstance()
          .addOdometryMeasurement(
              sampleTimestamps[updateIndex], m_gyroRotation, newModulePositions);
    }

    final ChassisSpeeds robotRelativeVelocity = swerveKinematics.toChassisSpeeds(getModuleStates());
    PoseEstimation.getInstance()
        .setVelocity(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond,
                robotRelativeVelocity.vyMetersPerSecond,
                m_imuInputs.connected
                    ? m_imuInputs.yawVelocityRadPerSec
                    : robotRelativeVelocity.omegaRadiansPerSecond));
  }

  public void runVelocity(ChassisSpeeds speeds) {
    if (m_headingController != null) speeds.omegaRadiansPerSecond += m_headingController.update();
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = swerveKinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, kMaxLinearSpeedMetersPerSecond);

    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int moduleIndex = 0; moduleIndex < m_modules.length; moduleIndex++) {
      optimizedSetpointStates[moduleIndex] =
          m_modules[moduleIndex].setSetpoint(setpointStates[moduleIndex]);
    }

    Logger.recordOutput("Drivetrain/SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("Drivetrain/SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  public void runWheelRadiusCharacterization(double characterizationInput) {
    runVelocity(new ChassisSpeeds(0.0, 0.0, characterizationInput));
  }

  public double[] getWheelRadiusCharacterizationPosition() {
    return Arrays.stream(m_modules).mapToDouble(Module::getDrivePositionRad).toArray();
  }

  public Command joystickDrive(
      DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
    UnaryOperator<Double> cubeAxis = axisMagnitude -> Math.pow(axisMagnitude, 3);
    return this.run(
            () -> {
              double x = cubeAxis.apply(MathUtil.applyDeadband(xSupplier.getAsDouble(), kDeadband));
              double y = cubeAxis.apply(MathUtil.applyDeadband(ySupplier.getAsDouble(), kDeadband));
              double omega =
                  cubeAxis.apply(MathUtil.applyDeadband(omegaSupplier.getAsDouble(), kDeadband));
              boolean isFlipped = Constants.onRedAllianceSupplier.getAsBoolean();
              runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      x * kMaxLinearSpeedMetersPerSecond,
                      y * kMaxLinearSpeedMetersPerSecond,
                      omega * kMaxAngularSpeedRadPerSec,
                      isFlipped
                          ? PoseEstimation.getInstance()
                              .getPose()
                              .getRotation()
                              .plus(new Rotation2d(Math.PI))
                          : PoseEstimation.getInstance().getPose().getRotation()));
            })
        .withName("Joystick Drive");
  }

  /**
   * This allows us to run auto-aim in auto when no other driving commands are active (no joystick
   * drive, no path following)
   */
  public Command blankDrive() {
    return this.run(() -> runVelocity(new ChassisSpeeds(0.0, 0.0, 0.0)));
  }

  public Command zeroGyro() {
    return Commands.runOnce(
            () ->
                resetPose.accept(
                    new Pose2d(
                        PoseEstimation.getInstance().getPose().getTranslation(),
                        Constants.onRedAllianceSupplier.getAsBoolean()
                            ? GeometryUtil.flipFieldRotation(new Rotation2d())
                            : new Rotation2d())))
        .ignoringDisable(true);
  }

  public Command sysIdDriveQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineDrive.quasistatic(direction);
  }

  public Command sysIdDriveDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineDrive.dynamic(direction);
  }

  public Command sysIdRotationQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineRotation.quasistatic(direction);
  }

  public Command sysIdRotationDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineRotation.dynamic(direction);
  }

  public Command sysIdSlipCurrent() {
    return new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.2).per(Seconds.of(1.0)),
                Volts.of(0.0),
                Seconds.of(30.0),
                (state) -> Logger.recordOutput("Drivetrain/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                  for (Module module : m_modules) {
                    module.runCharacterization(voltage.in(Volts));
                  }
                },
                null,
                this))
        .quasistatic(Direction.kForward);
  }

  private void configurePathing() {
    AutoBuilder.configureHolonomic(
        PoseEstimation.getInstance()::getPose,
        resetPose,
        () -> swerveKinematics.toChassisSpeeds(getModuleStates()),
        this::runVelocity,
        new HolonomicPathFollowerConfig(
            new PIDConstants(3.0, 0.0, 0.0),
            new PIDConstants(3.0, 0.0, 0.0),
            kMaxLinearSpeedMetersPerSecond,
            kDriveBaseRadius,
            new ReplanningConfig()),
        Constants.onRedAllianceSupplier,
        this);
    PathPlannerLogging.setLogActivePathCallback(
        activePath ->
            Logger.recordOutput("Drivetrain/Trajectory", activePath.toArray(Pose2d[]::new)));
    PathPlannerLogging.setLogTargetPoseCallback(
        targetPose -> Logger.recordOutput("Drivetrain/TrajectorySetpoint", targetPose));
  }

  @AutoLogOutput(key = "Drivetrain/SwerveStates/StatesMeasured")
  private SwerveModuleState[] getModuleStates() {
    return Arrays.stream(m_modules).map(Module::getState).toArray(SwerveModuleState[]::new);
  }

  @AutoLogOutput(key = "Drivetrain/SwerveStates/PositionsMeasured")
  private SwerveModulePosition[] getModulePositions() {
    return Arrays.stream(m_modules).map(Module::getPosition).toArray(SwerveModulePosition[]::new);
  }

  public Consumer<Pose2d> resetPose =
      (newPose) ->
          PoseEstimation.getInstance().resetPose(m_gyroRotation, getModulePositions(), newPose);

  public void setHeadingGoal(Supplier<Rotation2d> headingGoal) {
    m_headingController = new HeadingController(headingGoal);
  }

  public void clearHeadingGoal() {
    m_headingController = null;
    Logger.recordOutput("Drivetrain/HeadingController/Setpoint", new Pose2d());
  }

  @AutoLogOutput(key = "Drivetrain/HeadingController/AtGoal")
  public boolean atHeadingGoal() {
    return m_headingController != null && m_headingController.atGoal();
  }

  @AutoLogOutput(key = "Drivetrain/InRangeOfGoal")
  public boolean inRange() {
    return PoseEstimation.getInstance().getAimingParameters().effectiveDistance()
        <= speakerRangeMeters.get();
  }
}
