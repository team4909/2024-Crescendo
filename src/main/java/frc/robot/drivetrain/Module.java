package frc.robot.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

public class Module {

  // https://www.swervedrivespecialties.com/products/mk4i-swerve-module
  public static final double kDriveRatio = 6.75;
  public static final double kSteerRatio = 150.0 / 7.0;
  private final double kWheelDiameterMeters = Units.inchesToMeters(4.0);
  private final double kWheelRadiusMeters = kWheelDiameterMeters / 2.0;
  private final double kCouplingGearRatio = 50.0 / 14.0;

  private final ModuleIO m_io;
  private final ModuleIOInputsAutoLogged m_inputs = new ModuleIOInputsAutoLogged();
  private final int m_index;
  private SwerveModulePosition[] m_odometryPositions = new SwerveModulePosition[] {};

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
      var driveRotations = Units.radiansToRotations(m_inputs.odometryDrivePositionsRad[i]);
      var steerAngle = m_inputs.odometryTurnPositions[i];
      driveRotations -= steerAngle.getRotations() * kCouplingGearRatio;
      var driveRadians = Units.rotationsToRadians(driveRotations);
      double positionMeters = driveRadians / (kDriveRatio / kWheelRadiusMeters);
      m_odometryPositions[i] = new SwerveModulePosition(positionMeters, steerAngle);
    }
  }

  public SwerveModuleState setSetpoint(SwerveModuleState state) {
    final SwerveModuleState optimizedState =
        SwerveModuleState.optimize(state, m_inputs.steerPosition);
    double setpointVelocityRPS =
        Units.radiansToRotations(optimizedState.speedMetersPerSecond / kWheelRadiusMeters)
            * kDriveRatio;

    double angleError = optimizedState.angle.getRadians() - m_inputs.steerPosition.getRadians();
    setpointVelocityRPS *= Math.cos(angleError);

    var azimuthVelocityRPS = Units.radiansToRotations(m_inputs.steerVelocityRadPerSec);
    double driveRateBackOut = azimuthVelocityRPS *= kCouplingGearRatio;
    setpointVelocityRPS -= driveRateBackOut;

    Logger.recordOutput(
        "Test/Desired Speed Module " + m_index, optimizedState.speedMetersPerSecond);
    m_io.setSteerRotations(optimizedState.angle.getRotations());
    m_io.setDriveRPS(setpointVelocityRPS);
    return state;
  }

  public void runCharacterization(double volts) {
    m_io.setSteerRotations(new Rotation2d().getRotations());
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

  // getOdometryPositions() should be used for performant odometry updates, not this. NOTE that this
  // also does not compensate for drive-azimuth coupling.
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_inputs.drivePositionRad * kWheelRadiusMeters, m_inputs.steerPosition);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_inputs.driveVelocityRadPerSec * kWheelRadiusMeters, m_inputs.steerPosition);
  }
}
