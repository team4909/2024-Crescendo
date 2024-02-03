package frc.robot.arm;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.system.NumericalIntegration;
import frc.robot.arm.ArmConfig.JointConfig;

/**
 * Calculates feedforward currents for a double jointed arm.
 *
 * <p>https://www.chiefdelphi.com/t/whitepaper-two-jointed-arm-dynamics/423060 Builds off of the
 * work done by Teams 6328 and 3467 in 2023.
 */
public class ArmModel {
  private static final double g = 9.80665;

  private final JointConfig m_elbowJointConfig;
  private final JointConfig m_wristJointConfig;

  public ArmModel(JointConfig elbowJointConfig, JointConfig wristJointConfig) {
    this.m_elbowJointConfig = elbowJointConfig;
    this.m_wristJointConfig = wristJointConfig;
  }

  public Vector<N2> feedforward(Vector<N2> position) {
    return feedforward(position, VecBuilder.fill(0.0, 0.0), VecBuilder.fill(0.0, 0.0));
  }

  public Vector<N2> feedforward(Vector<N2> position, Vector<N2> velocity, Vector<N2> acceleration) {
    var torque =
        M(position)
            .times(acceleration)
            .plus(C(position, velocity).times(velocity))
            .plus(Tg(position));
    return VecBuilder.fill(
        m_elbowJointConfig.gearbox().getVoltage(torque.get(0, 0), velocity.get(0, 0)),
        m_wristJointConfig.gearbox().getVoltage(torque.get(1, 0), velocity.get(1, 0)));
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
    return new Vector<>(
        NumericalIntegration.rkdp(
            (Matrix<N4, N1> x, Matrix<N2, N1> u) -> {
              // x = current state, u = voltages, return = state derivatives

              // Get vectors from state
              var position = VecBuilder.fill(x.get(0, 0), x.get(1, 0));
              var velocity = VecBuilder.fill(x.get(2, 0), x.get(3, 0));

              // Calculate torque
              var elbowTorque =
                  m_elbowJointConfig
                      .gearbox()
                      .getTorque(
                          m_elbowJointConfig.gearbox().getCurrent(velocity.get(0, 0), u.get(0, 0)));
              var wristTorque =
                  m_wristJointConfig
                      .gearbox()
                      .getTorque(
                          m_wristJointConfig.gearbox().getCurrent(velocity.get(1, 0), u.get(1, 0)));
              var torque = VecBuilder.fill(elbowTorque, wristTorque);

              // Apply limits
              if (position.get(0, 0) < m_elbowJointConfig.minAngle()) {
                position.set(0, 0, m_elbowJointConfig.minAngle());
                if (velocity.get(0, 0) < 0.0) {
                  velocity.set(0, 0, 0.0);
                }
                if (torque.get(0, 0) < 0.0) {
                  torque.set(0, 0, 0.0);
                }
              }
              if (position.get(0, 0) > m_elbowJointConfig.maxAngle()) {
                position.set(0, 0, m_elbowJointConfig.maxAngle());
                if (velocity.get(0, 0) > 0.0) {
                  velocity.set(0, 0, 0.0);
                }
                if (torque.get(0, 0) > 0.0) {
                  torque.set(0, 0, 0.0);
                }
              }
              if (position.get(1, 0) < m_wristJointConfig.minAngle()) {
                position.set(1, 0, m_wristJointConfig.minAngle());
                if (velocity.get(1, 0) < 0.0) {
                  velocity.set(1, 0, 0.0);
                }
                if (torque.get(1, 0) < 0.0) {
                  torque.set(1, 0, 0.0);
                }
              }
              if (position.get(1, 0) > m_wristJointConfig.maxAngle()) {
                position.set(1, 0, m_wristJointConfig.maxAngle());
                if (velocity.get(1, 0) > 0.0) {
                  velocity.set(1, 0, 0.0);
                }
                if (torque.get(1, 0) > 0.0) {
                  torque.set(1, 0, 0.0);
                }
              }

              // Calculate acceleration
              var acceleration =
                  M(position)
                      .inv()
                      .times(
                          torque.minus(C(position, velocity).times(velocity)).minus(Tg(position)));

              // Return state vector
              return MatBuilder.fill(
                  Nat.N4(),
                  Nat.N1(),
                  velocity.get(0, 0),
                  velocity.get(1, 0),
                  acceleration.get(0, 0),
                  acceleration.get(1, 0));
            },
            state,
            voltage,
            dt));
  }

  private Matrix<N2, N2> M(Vector<N2> position) {
    var M = new Matrix<>(N2.instance, N2.instance);
    M.set(
        0,
        0,
        m_elbowJointConfig.mass() * Math.pow(m_elbowJointConfig.cgRadius(), 2.0)
            + m_wristJointConfig.mass()
                * (Math.pow(m_elbowJointConfig.length(), 2.0)
                    + Math.pow(m_wristJointConfig.cgRadius(), 2.0))
            + m_elbowJointConfig.moi()
            + m_wristJointConfig.moi()
            + 2
                * m_wristJointConfig.mass()
                * m_elbowJointConfig.length()
                * m_wristJointConfig.cgRadius()
                * Math.cos(position.get(1, 0)));
    M.set(
        1,
        0,
        m_wristJointConfig.mass() * Math.pow(m_wristJointConfig.cgRadius(), 2.0)
            + m_wristJointConfig.moi()
            + m_wristJointConfig.mass()
                * m_elbowJointConfig.length()
                * m_wristJointConfig.cgRadius()
                * Math.cos(position.get(1, 0)));
    M.set(
        0,
        1,
        m_wristJointConfig.mass() * Math.pow(m_wristJointConfig.cgRadius(), 2.0)
            + m_wristJointConfig.moi()
            + m_wristJointConfig.mass()
                * m_elbowJointConfig.length()
                * m_wristJointConfig.cgRadius()
                * Math.cos(position.get(1, 0)));
    M.set(
        1,
        1,
        m_wristJointConfig.mass() * Math.pow(m_wristJointConfig.cgRadius(), 2.0)
            + m_wristJointConfig.moi());
    return M;
  }

  private Matrix<N2, N2> C(Vector<N2> position, Vector<N2> velocity) {
    var C = new Matrix<>(N2.instance, N2.instance);
    C.set(
        0,
        0,
        -m_wristJointConfig.mass()
            * m_elbowJointConfig.length()
            * m_wristJointConfig.cgRadius()
            * Math.sin(position.get(1, 0))
            * velocity.get(1, 0));
    C.set(
        1,
        0,
        m_wristJointConfig.mass()
            * m_elbowJointConfig.length()
            * m_wristJointConfig.cgRadius()
            * Math.sin(position.get(1, 0))
            * velocity.get(0, 0));
    C.set(
        0,
        1,
        -m_wristJointConfig.mass()
            * m_elbowJointConfig.length()
            * m_wristJointConfig.cgRadius()
            * Math.sin(position.get(1, 0))
            * (velocity.get(0, 0) + velocity.get(1, 0)));
    return C;
  }

  private Matrix<N2, N1> Tg(Vector<N2> position) {
    var Tg = new Matrix<>(N2.instance, N1.instance);
    Tg.set(
        0,
        0,
        (m_elbowJointConfig.mass() * m_elbowJointConfig.cgRadius()
                    + m_wristJointConfig.mass() * m_elbowJointConfig.length())
                * g
                * Math.cos(position.get(0, 0))
            + m_wristJointConfig.mass()
                * m_wristJointConfig.cgRadius()
                * g
                * Math.cos(position.get(0, 0) + position.get(1, 0)));
    Tg.set(
        1,
        0,
        m_wristJointConfig.mass()
            * m_wristJointConfig.cgRadius()
            * g
            * Math.cos(position.get(0, 0) + position.get(1, 0)));
    return Tg;
  }
}
