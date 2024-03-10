package frc.robot.drivetrain;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import java.util.Queue;

public class ImuIOPigeon2 implements ImuIO {
  private final Pigeon2 m_imu = new Pigeon2(20, Constants.kDrivetrainCanBus);
  private final StatusSignal<Double> m_yawSignal = m_imu.getYaw();
  private final StatusSignal<Double> m_yawVelocitySignal = m_imu.getAngularVelocityZWorld();
  private final Queue<Double> m_yawPositionQueue, m_yawTimestampQueue;

  public ImuIOPigeon2() {
    m_imu.getConfigurator().apply(new Pigeon2Configuration());
    m_imu.getConfigurator().setYaw(0.0);
    m_yawSignal.setUpdateFrequency(PhoenixOdometryThread.kOdometryFrequencyHz);
    m_yawVelocitySignal.setUpdateFrequency(100.0);
    m_imu.optimizeBusUtilization();
    m_yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(m_imu, m_imu.getYaw());
    m_yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
  }

  @Override
  public void updateInputs(ImuIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(m_yawSignal, m_yawVelocitySignal).equals(StatusCode.OK);
    inputs.yawPosition = Rotation2d.fromDegrees(m_yawSignal.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(m_yawVelocitySignal.getValueAsDouble());

    inputs.odometryYawPositions =
        m_yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    inputs.odometryYawTimestamps =
        m_yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    m_yawPositionQueue.clear();
    m_yawTimestampQueue.clear();
  }
}
