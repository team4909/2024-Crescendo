package frc.robot.arm;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmModel {

  public static final Translation2d origin =
      new Translation2d(Units.inchesToMeters(-9.2), Units.inchesToMeters(20));

  public static final double kElbowGearboxReduction = 3.0 * 5.0 * 5.0;
  public static final double kElbowChainReduction = 36.0 / 22.0;
  public static final double kElbowFinalReduction = kElbowGearboxReduction * kElbowChainReduction;
  public static final double kElbowLengthMeters = Units.inchesToMeters(19.0);
  public static final double kElbowMinAngleRad = -Math.PI;
  public static final double kElbowMaxAngleRad = Math.PI;
  public static final DCMotor kElbowGearbox = DCMotor.getFalcon500Foc(2);
  private final double kElbowMassKg = Units.lbsToKilograms(7.0);
  private final double kElbowMoiKgMetersSq =
      SingleJointedArmSim.estimateMOI(kElbowLengthMeters, kElbowMassKg);

  public static final double kWristGearboxReduction = 3.0 * 5.0;
  public static final double kWristChainReduction = 36.0 / 22.0;
  public static final double kWristFinalReduction = kWristGearboxReduction * kWristChainReduction;
  public static final double kWristLengthMeters = Units.inchesToMeters(15.0);
  public static final double kWristMinAngleRad = 2.0 * -Math.PI;
  public static final double kWristMaxAngleRad = 2.0 * Math.PI;
  public static final DCMotor kWristGearbox = DCMotor.getFalcon500Foc(2);
  private final double kWristMassKg = Units.lbsToKilograms(11.5);
  private final double kWristMoiKgMetersSq =
      SingleJointedArmSim.estimateMOI(kWristLengthMeters, kWristMassKg);
  private final SingleJointedArmSim m_elbowSim, m_wristSim;

  public ArmModel() {
    m_elbowSim =
        new SingleJointedArmSim(
            kElbowGearbox,
            kElbowFinalReduction,
            kElbowMoiKgMetersSq,
            kElbowLengthMeters,
            kElbowMinAngleRad,
            kElbowMaxAngleRad,
            true,
            0.0);
    m_wristSim =
        new SingleJointedArmSim(
            kWristGearbox,
            kWristFinalReduction,
            kWristMoiKgMetersSq,
            kWristLengthMeters,
            kWristMinAngleRad,
            kWristMaxAngleRad,
            true,
            0.0);
  }

  /**
   * Adjusts the simulated state of the arm based on applied voltages.
   *
   * @param state The current state of the arm as (position_0, position_1, velocity_0, velocity_1)
   * @param voltage The applied voltage of each joint.
   * @param dt The step length in seconds.
   * @return The new state of the arm as (position_0, position_1, velocity_0, velocity_1)
   */
  public Vector<N4> simulate(Vector<N4> state, Vector<N2> voltage, double dt) {
    m_elbowSim.setState(state.get(0, 0), state.get(2, 0));
    m_wristSim.setState(state.get(1, 0), state.get(3, 0));
    m_elbowSim.setInputVoltage(voltage.get(0, 0));
    m_wristSim.setInputVoltage(voltage.get(1, 0));
    m_elbowSim.update(dt);
    m_wristSim.update(dt);
    return VecBuilder.fill(
        m_elbowSim.getAngleRads(),
        m_wristSim.getAngleRads(),
        m_elbowSim.getVelocityRadPerSec(),
        m_wristSim.getVelocityRadPerSec());
  }
}
