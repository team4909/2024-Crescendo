package frc.robot.drivetrain;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class PhoenixOdometryThread extends Thread {

  public static final double kOdometryFrequencyHz = 250.0;
  private final Lock m_signalsLock = new ReentrantLock();
  private BaseStatusSignal[] m_signals = new BaseStatusSignal[0];
  // State is defined as <position, velocity>
  private final List<Pair<StatusSignal<Double>, StatusSignal<Double>>> allStateSignals =
      new ArrayList<>();
  private final List<Queue<Double>> m_positionQueues = new ArrayList<>();
  private final List<Queue<Double>> m_timestampQueues = new ArrayList<>();

  private double m_averageLoopTimeMs = 0.0;
  public final Supplier<Double> averageLoopTimeSupplier = () -> m_averageLoopTimeMs;

  private static PhoenixOdometryThread instance = null;

  public static PhoenixOdometryThread getInstance() {
    if (instance == null) {
      instance = new PhoenixOdometryThread();
    }
    return instance;
  }

  private PhoenixOdometryThread() {
    setName("PhoenixOdometryThread");
    setDaemon(true);
    start();
  }

  @Override
  public void start() {
    if (m_timestampQueues.size() > 0) super.start();
  }

  public Queue<Double> registerSignals(
      ParentDevice device, StatusSignal<Double> signal, StatusSignal<Double> signalSlope) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    m_signalsLock.lock();
    Drivetrain.odometryLock.lock();
    try {
      BaseStatusSignal[] newSignals = new BaseStatusSignal[m_signals.length + 2];
      System.arraycopy(m_signals, 0, newSignals, 0, m_signals.length);
      newSignals[m_signals.length] = signal;
      newSignals[m_signals.length + 1] = signalSlope;
      m_signals = newSignals;
      m_positionQueues.add(queue);
      allStateSignals.add(new Pair<>(signal, signalSlope));
    } finally {
      m_signalsLock.unlock();
      Drivetrain.odometryLock.unlock();
    }
    return queue;
  }

  public Queue<Double> makeTimestampQueue() {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Drivetrain.odometryLock.lock();
    try {
      m_timestampQueues.add(queue);
    } finally {
      Drivetrain.odometryLock.unlock();
    }
    return queue;
  }

  @Override
  public void run() {
    final MedianFilter peakRemover = new MedianFilter(3);
    final LinearFilter lowPassFilter = LinearFilter.movingAverage(50);
    double lastTime = 0.0;
    double currentTime = 0.0;
    while (true) {
      m_signalsLock.lock();
      try {
        BaseStatusSignal.waitForAll(2.0 / kOdometryFrequencyHz, m_signals);
      } finally {
        m_signalsLock.unlock();
      }
      Drivetrain.odometryLock.lock();
      try {
        lastTime = currentTime;
        currentTime = Utils.getCurrentTimeSeconds();
        m_averageLoopTimeMs =
            lowPassFilter.calculate(peakRemover.calculate(currentTime - lastTime)) * 1e3;

        double timestamp = Logger.getRealTimestamp() / 1e6;
        double totalLatency = 0.0;
        for (BaseStatusSignal signal : m_signals)
          totalLatency += signal.getTimestamp().getLatency();
        if (m_signals.length > 0) timestamp -= totalLatency / m_signals.length;

        for (int i = 0; i < m_positionQueues.size(); i++) {
          final Pair<StatusSignal<Double>, StatusSignal<Double>> stateSignals =
              allStateSignals.get(i);
          m_positionQueues
              .get(i)
              .offer(
                  BaseStatusSignal.getLatencyCompensatedValue(
                      stateSignals.getFirst(), stateSignals.getSecond()));
        }
        for (int i = 0; i < m_timestampQueues.size(); i++)
          m_timestampQueues.get(i).offer(timestamp);
      } finally {
        Drivetrain.odometryLock.unlock();
      }
    }
  }
}
