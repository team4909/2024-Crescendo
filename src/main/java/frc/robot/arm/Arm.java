package frc.robot.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.arm.ArmConfig.JointConfig;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {

  private final ArmIO m_io;
  private final ArmIOInputsAutoLogged m_inputs = new ArmIOInputsAutoLogged();
  private final ArmModel m_armModel = new ArmModel(ArmConfig.kElbowConfig, ArmConfig.kWristConfig);
  private final ArmKinematics m_armKinematics = new ArmKinematics();
  private final ArmVisualizer m_measuredVisualizer;
  private final ArmVisualizer m_setpointVisualizer;

  public Arm(ArmIO io) {
    m_io = io;
    m_measuredVisualizer = new ArmVisualizer("ArmMeasured");
    m_setpointVisualizer = new ArmVisualizer("ArmSetpoint");
  }

  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Arm", m_inputs);
    Logger.recordOutput(
        "Arm/Cartesian Position",
        m_armKinematics.forward(
            VecBuilder.fill(m_inputs.elbowPositionRad, m_inputs.wristPositionRad)));

    m_measuredVisualizer.update(m_inputs.elbowPositionRad, m_inputs.wristPositionRad);
  }

  public void setSetpoint(Translation2d setpoint) {
    Optional<Vector<N2>> setpointAnglesRad = m_armKinematics.inverse(setpoint);
    setpointAnglesRad.ifPresent(
        angles -> {
          double elbowAngle = angles.get(0, 0);
          double wristAngle = angles.get(1, 0);
          Logger.recordOutput("Arm/Goal Elbow Angle", (elbowAngle));
          Logger.recordOutput("Arm/Goal Wrist Angle", (wristAngle));
          m_setpointVisualizer.update(elbowAngle, wristAngle);
          Vector<N2> feedforwardVolts = m_armModel.feedforward(angles);
          double elbowFeedForward = feedforwardVolts.get(0, 0);
          double wristFeedForward = feedforwardVolts.get(1, 0);
          Logger.recordOutput("Arm/Elbow Feed Forward", elbowFeedForward);
          Logger.recordOutput("Arm/Wrist Feed Forward", wristFeedForward);
          m_io.setElbowRotatations(Units.radiansToRotations(elbowAngle), elbowFeedForward);
          m_io.setWristRotatations(Units.radiansToRotations(wristAngle), wristFeedForward);
        });
  }

  // public Command testSetpoint() {
  //   return this.run(() -> setSetpoint(new Translation2d(0.5, 0.6))).withName("Test Setpoint");
  // }

  Translation2d folded = new Translation2d(0.0762, 0.4699);
  Translation2d trapSetpoint =
      new Translation2d(Units.inchesToMeters(19.8), Units.inchesToMeters(30));

  public Command testSetpoint() {
    return this.run(() -> setSetpoint(trapSetpoint)).withName("Test Setpoint");
  }

  public Command stop() {
    return this.run(
            () -> {
              m_io.setElbowVoltage(0.0);
              m_io.setWristVoltage(0.0);
            })
        .withName("Stop");
  }

  class ArmKinematics {

    private final JointConfig m_elbowConfig = ArmConfig.kElbowConfig;
    private final JointConfig m_wristConfig = ArmConfig.kWristConfig;

    public Translation2d forward(Vector<N2> angles) {

      return new Translation2d(
          ArmConfig.kOrigin.getX()
              + m_elbowConfig.length() * Math.cos(angles.get(0, 0))
              + m_wristConfig.length() * Math.cos(angles.get(0, 0) + angles.get(1, 0)),
          ArmConfig.kOrigin.getY()
              + m_elbowConfig.length() * Math.sin(angles.get(0, 0))
              + m_wristConfig.length() * Math.sin(angles.get(0, 0) + angles.get(1, 0)));
    }

    public Optional<Vector<N2>> inverse(Translation2d position) {
      Translation2d relativePosition = position.minus(ArmConfig.kOrigin);

      // Flip when X is negative
      // boolean isFlipped = relativePosition.getX() < 0.0;
      // if (isFlipped) {
      //   relativePosition = new Translation2d(-relativePosition.getX(), relativePosition.getY());
      // }

      // Calculate angles
      double wristAngle =
          -Math.acos(
              (Math.pow(relativePosition.getX(), 2)
                      + Math.pow(relativePosition.getY(), 2)
                      - Math.pow(m_elbowConfig.length(), 2)
                      - Math.pow(m_wristConfig.length(), 2))
                  / (2 * m_elbowConfig.length() * m_wristConfig.length()));
      if (Double.isNaN(wristAngle)) {
        return Optional.empty();
      }
      double elbowAngle =
          Math.atan(relativePosition.getY() / relativePosition.getX())
              - Math.atan(
                  (m_wristConfig.length() * Math.sin(wristAngle))
                      / (m_elbowConfig.length() + m_wristConfig.length() * Math.cos(wristAngle)));

      // Invert shoulder angle if invalid
      Translation2d testPosition =
          forward(VecBuilder.fill(elbowAngle, wristAngle)).minus(ArmConfig.kOrigin);
      if (testPosition.getDistance(relativePosition) > 1e-3) {
        elbowAngle += Math.PI;
      }

      // Flip angles
      // if (isFlipped) {
      //   elbowAngle = Math.PI - elbowAngle;
      //   wristAngle = -wristAngle;
      // }

      // Wrap angles to correct ranges
      elbowAngle = MathUtil.inputModulus(elbowAngle, 0, 2 * Math.PI);
      wristAngle = MathUtil.inputModulus(wristAngle, 0, 2 * Math.PI);

      // Exit if outside valid ranges for the joints
      if (elbowAngle < m_elbowConfig.minAngle()
          || elbowAngle > m_elbowConfig.maxAngle()
          || wristAngle < m_wristConfig.minAngle()
          || wristAngle > m_wristConfig.maxAngle()) {
        return Optional.empty();
      }
      return Optional.of(VecBuilder.fill(Units.degreesToRadians(0), Units.degreesToRadians(0)));
      // return Optional.of(VecBuilder.fill(elbowAngle, wristAngle));
    }
  }
}
