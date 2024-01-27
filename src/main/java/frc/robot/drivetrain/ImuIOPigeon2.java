package frc.robot.drivetrain;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import java.util.Queue;

public class ImuIOPigeon2 implements ImuIO {
  private final Pigeon2 m_imu = new Pigeon2(20, "CANivore1");
  private final StatusSignal<Double> yaw = m_imu.getYaw();
  private final Queue<Double> m_yawPositionQueue, m_yawTimestampQueue;
  private final StatusSignal<Double> yawVelocity = m_imu.getAngularVelocityZWorld();

  public ImuIOPigeon2() {
    m_imu.getConfigurator().apply(new Pigeon2Configuration());
    m_imu.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(Module.kOdometryFrequencyHz);
    yawVelocity.setUpdateFrequency(100.0);
    m_imu.optimizeBusUtilization();
    m_yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(m_imu, m_imu.getYaw());
    m_yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
  }

  @Override
  public void updateInputs(ImuIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    inputs.odometryYawPositions =
        m_yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    inputs.odometryYawTimestamps =
        m_yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    m_yawPositionQueue.clear();
    m_yawTimestampQueue.clear();
  }

  public void setGyroAngle(double angleRad) {
    m_imu.setYaw(angleRad);
  }

  public void updateSim(double dThetaRad) {
    final Pigeon2SimState imuSimState = m_imu.getSimState();
    imuSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    imuSimState.addYaw(Math.toDegrees(dThetaRad));
  }
}
