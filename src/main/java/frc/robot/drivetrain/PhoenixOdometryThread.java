package frc.robot.drivetrain;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.Logger;

public class PhoenixOdometryThread extends Thread {

  public static final double kOdometryFrequencyHz = 250.0;
  private final Lock m_signalsLock = new ReentrantLock();
  private BaseStatusSignal[] m_signals = new BaseStatusSignal[0];
  private final List<Queue<Double>> m_queues = new ArrayList<>();
  private final List<Queue<Double>> m_timestampQueues = new ArrayList<>();

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
  }

  @Override
  public void start() {
    if (m_timestampQueues.size() > 0) {
      super.start();
    }
  }

  public Queue<Double> registerSignal(ParentDevice device, StatusSignal<Double> signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    m_signalsLock.lock();
    Drivetrain.odometryLock.lock();
    try {
      BaseStatusSignal[] newSignals = new BaseStatusSignal[m_signals.length + 1];
      System.arraycopy(m_signals, 0, newSignals, 0, m_signals.length);
      newSignals[m_signals.length] = signal;
      m_signals = newSignals;
      m_queues.add(queue);
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
    while (true) {
      m_signalsLock.lock();
      try {
        BaseStatusSignal.waitForAll(2.0 / kOdometryFrequencyHz, m_signals);
      } finally {
        m_signalsLock.unlock();
      }
      Drivetrain.odometryLock.lock();
      try {
        double timestamp = Logger.getRealTimestamp() / 1e6;
        double totalLatency = 0.0;
        for (BaseStatusSignal signal : m_signals) {
          totalLatency += signal.getTimestamp().getLatency();
        }
        if (m_signals.length > 0) {
          timestamp -= totalLatency / m_signals.length;
        }
        for (int i = 0; i < m_signals.length; i++) {
          m_queues.get(i).offer(m_signals[i].getValueAsDouble());
        }
        for (int i = 0; i < m_timestampQueues.size(); i++) {
          m_timestampQueues.get(i).offer(timestamp);
        }
      } finally {
        Drivetrain.odometryLock.unlock();
      }
    }
  }
}
