package frc.robot.feeder;

import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SynchronousInterrupt;
import java.util.function.BooleanSupplier;

public class SensorThread {
  private final double kUpdateFrequencyHz = 200.0;
  private final Notifier m_thread;
  private final SynchronousInterrupt m_synchronousInterrupt;
  private double m_lastRisingEdgeTimestamp = 0.0;
  private double m_lastFallingEdgeTimestamp = 0.0;
  private boolean m_sensorTripped = false;
  public final BooleanSupplier getSensorTripped = () -> m_sensorTripped;

  public SensorThread(DigitalSource digitalSource) {
    m_thread = new Notifier(this::periodic);
    m_thread.setName("SensorThread");
    m_synchronousInterrupt = new SynchronousInterrupt(digitalSource);
    m_synchronousInterrupt.setInterruptEdges(true, true);
  }

  public void start() {
    m_thread.startPeriodic(1.0 / kUpdateFrequencyHz);
  }

  private void periodic() {
    final double latestRisingEdgeTimestamp = m_synchronousInterrupt.getRisingTimestamp();
    final double latestFallingEdgeTimestamp = m_synchronousInterrupt.getFallingTimestamp();
    final boolean newRisingEdge = latestRisingEdgeTimestamp > m_lastRisingEdgeTimestamp;
    if (newRisingEdge) m_lastRisingEdgeTimestamp = latestRisingEdgeTimestamp;
    final boolean newFallingEdge = latestFallingEdgeTimestamp > m_lastFallingEdgeTimestamp;
    if (newFallingEdge) m_lastFallingEdgeTimestamp = latestFallingEdgeTimestamp;
    if (latestFallingEdgeTimestamp > latestRisingEdgeTimestamp && newFallingEdge)
      m_sensorTripped = true;
    else if (latestRisingEdgeTimestamp > latestFallingEdgeTimestamp && newRisingEdge)
      m_sensorTripped = false;
  }
}
