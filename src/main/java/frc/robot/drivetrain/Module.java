package frc.robot.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Module {

  // https://www.swervedrivespecialties.com/products/mk4i-swerve-module
  // For Viper our first stage is 50:13 instead of 50:14 because we have a different pinion.
  private static final double kDriveRatioFirstStage =
      Constants.kIsViper ? (50.0 / 13.0) : (50.0 / 14.0);
  public static final double kDriveRatio = kDriveRatioFirstStage * (16.0 / 28.0) * (45.0 / 15.0);
  public static final double kSteerRatio = 150.0 / 7.0;
  private final double kWheelDiameterMeters = Units.inchesToMeters(1.73 * 2);
  private final double kWheelRadiusMeters = kWheelDiameterMeters / 2.0;
  private final double kCouplingRatio = kDriveRatioFirstStage;

  private final ModuleIO m_io;
  private final ModuleIOInputsAutoLogged m_inputs = new ModuleIOInputsAutoLogged();
  private final int m_index;
  private SwerveModulePosition[] m_odometryPositions = new SwerveModulePosition[] {};
  private SwerveModulePosition m_lastPosition = new SwerveModulePosition();

  public Module(ModuleIO io, int index) {
    m_io = io;
    m_index = index;
  }

  public void updateInputs() {
    m_io.updateInputs(m_inputs);
  }

  public void periodic() {
    Logger.processInputs("DrivetrainInputs/Module" + m_index, m_inputs);

    int sampleCount = m_inputs.odometryTimestamps.length;
    m_odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double driveRotations = Units.radiansToRotations(m_inputs.odometryDrivePositionsRad[i]);
      Rotation2d steerAngle = m_inputs.odometryTurnPositions[i];
      driveRotations -= steerAngle.getRotations() * kCouplingRatio;
      double driveRadians = Units.rotationsToRadians(driveRotations);
      double positionMeters = driveRadians * kWheelRadiusMeters;
      m_odometryPositions[i] = new SwerveModulePosition(positionMeters, steerAngle);
      m_lastPosition = m_odometryPositions[i];
    }
  }

  public SwerveModuleState setSetpoint(SwerveModuleState state) {
    final SwerveModuleState optimizedState =
        SwerveModuleState.optimize(state, m_inputs.steerPosition);
    double setpointVelocityRPS =
        Units.radiansToRotations(optimizedState.speedMetersPerSecond / kWheelRadiusMeters);

    double angleError = optimizedState.angle.getRadians() - m_inputs.steerPosition.getRadians();
    setpointVelocityRPS *= Math.max(0.0, Math.cos(angleError));

    double azimuthVelocityRPS = Units.radiansToRotations(m_inputs.steerVelocityRadPerSec);
    double driveRateBackOut = azimuthVelocityRPS * kCouplingRatio;
    setpointVelocityRPS += driveRateBackOut;

    m_io.setSteerRotations(optimizedState.angle.getRotations());
    m_io.setDriveRPS(setpointVelocityRPS);
    return state;
  }

  public void runCharacterization(double volts) {
    m_io.setSteerRotations(new Rotation2d().getRotations());
    m_io.setDriveVoltage(volts);
  }

  public void runCharacterization(Rotation2d angle, double volts) {
    m_io.setSteerRotations(angle.getRotations());
    m_io.setDriveVoltage(volts);
  }

  public void stop() {
    m_io.setDriveVoltage(0.0);
    m_io.setSteerVoltage(0.0);
  }

  public SwerveModulePosition[] getOdometryPositions() {
    return m_odometryPositions;
  }

  public double[] getOdometryTimestamps() {
    return m_inputs.odometryTimestamps;
  }

  public SwerveModulePosition getPosition() {
    return m_lastPosition;
  }

  public double getDrivePositionRad() {
    return m_inputs.drivePositionRad;
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_inputs.driveVelocityRadPerSec * kWheelRadiusMeters, m_inputs.steerPosition);
  }
}
