package frc.robot.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.LoggedTunableNumber;
import frc.robot.Constants;
import java.util.Optional;
import java.util.function.BiFunction;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {

  private final double kSDeadband = 0.05;
  private final ArmIO m_io;
  private final ArmIOInputsAutoLogged m_inputs = new ArmIOInputsAutoLogged();
  private final ArmModel m_armModel = new ArmModel();
  private final ArmKinematics m_kinematics = new ArmKinematics();
  private final ArmVisualizer m_measuredVisualizer;
  private final ArmVisualizer m_setpointVisualizer;
  private ProfiledPIDController m_elbowController =
      new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));
  private ProfiledPIDController m_wristController =
      new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));

  private static final LoggedTunableNumber elbowkP = new LoggedTunableNumber("Arm/Elbow/kP");
  private static final LoggedTunableNumber elbowkD = new LoggedTunableNumber("Arm/Elbow/kD");
  private static final LoggedTunableNumber elbowkS = new LoggedTunableNumber("Arm/Elbow/kS");
  private static final LoggedTunableNumber elbowCruiseVelocityRadPerSec =
      new LoggedTunableNumber("Arm/Elbow/CruiseVelocity");
  private static final LoggedTunableNumber elbowMaxAccelerationRadPerSecSq =
      new LoggedTunableNumber("Arm/Elbow/MaxAcceleration");
  private static final LoggedTunableNumber wristkP = new LoggedTunableNumber("Arm/Wrist/kP");
  private static final LoggedTunableNumber wristkD = new LoggedTunableNumber("Arm/Wrist/kD");
  private static final LoggedTunableNumber wristkS = new LoggedTunableNumber("Arm/Wrist/kS");
  private static final LoggedTunableNumber wristCruiseVelocityRadPerSec =
      new LoggedTunableNumber("Arm/Wrist/CruiseVelocity");
  private static final LoggedTunableNumber wristMaxAccelerationRadPerSecSq =
      new LoggedTunableNumber("Arm/Wrist/MaxAcceleration");

  // We don't want to add kS to the feedback when the output is close to 0
  private final BiFunction<Double, Double, Double> m_deadbandkS =
      (volts, kS) -> Math.abs(volts) < kSDeadband ? volts : volts + Math.copySign(kS, volts);

  private Vector<N2> m_profileInitialAngles;
  private double m_elbowPositionRad, m_wristPositionRad;

  static {
    switch (Constants.kCurrentMode) {
      case kReal:
        elbowkP.initDefault(3.8);
        elbowkD.initDefault(0);
        elbowkS.initDefault(0.55);
        elbowCruiseVelocityRadPerSec.initDefault(5.0);
        elbowMaxAccelerationRadPerSecSq.initDefault(8.0);
        wristkP.initDefault(3.3);
        wristkD.initDefault(0);
        wristkS.initDefault(0.4);
        wristCruiseVelocityRadPerSec.initDefault(4.0);
        wristMaxAccelerationRadPerSecSq.initDefault(8.0);
        break;
      case kSim:
        // We do not want kS in simulation
        elbowkP.initDefault(5.0);
        elbowkD.initDefault(0.0);
        elbowCruiseVelocityRadPerSec.initDefault(3.0);
        elbowMaxAccelerationRadPerSecSq.initDefault(2.0);
        wristkP.initDefault(20.0);
        wristkD.initDefault(0.0);
        wristCruiseVelocityRadPerSec.initDefault(5.0);
        wristMaxAccelerationRadPerSecSq.initDefault(2.0);
        break;
      default:
        break;
    }
  }

  public Arm(ArmIO io) {
    m_io = io;
    m_measuredVisualizer = new ArmVisualizer("ArmMeasured");
    m_setpointVisualizer = new ArmVisualizer("ArmSetpoint");
    m_wristController.enableContinuousInput(-Math.PI, Math.PI);
    m_elbowController.setTolerance(Units.degreesToRadians(0.25));
    m_wristController.setTolerance(Units.degreesToRadians(0.25));
    setDefaultCommand(idle());
  }

  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("ArmInputs", m_inputs);

    m_elbowPositionRad = m_inputs.elbowRelativePositionRad;
    m_wristPositionRad = m_inputs.wristRelativePositionRad;

    if (DriverStation.isDisabled()) {
      m_io.setElbowVoltage(0.0);
      m_io.setWristVoltage(0.0);
    }

    // #region Update controllers wßhen tunables change.
    if (elbowkP.hasChanged(hashCode()) || elbowkD.hasChanged(hashCode())) {
      m_elbowController.setP(elbowkP.get());
      m_elbowController.setD(elbowkD.get());
    }
    if (wristkP.hasChanged(hashCode()) || wristkD.hasChanged(hashCode())) {
      m_wristController.setP(wristkP.get());
      m_wristController.setD(wristkD.get());
    }

    if (elbowCruiseVelocityRadPerSec.hasChanged(hashCode())
        || elbowMaxAccelerationRadPerSecSq.hasChanged(hashCode())) {
      m_elbowController.setConstraints(
          new TrapezoidProfile.Constraints(
              elbowCruiseVelocityRadPerSec.get(), elbowMaxAccelerationRadPerSecSq.get()));
    }
    if (wristCruiseVelocityRadPerSec.hasChanged(hashCode())
        || wristMaxAccelerationRadPerSecSq.hasChanged(hashCode())) {
      m_wristController.setConstraints(
          new TrapezoidProfile.Constraints(
              wristCruiseVelocityRadPerSec.get(), wristMaxAccelerationRadPerSecSq.get()));
    }
    // #endregion

    Logger.recordOutput(
        "Arm/Forward Kinematics Position",
        m_kinematics.forward(VecBuilder.fill(m_elbowPositionRad, m_wristPositionRad)));
    m_measuredVisualizer.update(m_elbowPositionRad, m_wristPositionRad);
    Logger.recordOutput(
        "Arm/Current Command",
        getCurrentCommand() == null ? "Null" : getCurrentCommand().getName());
  }

  private void setSetpoint(Vector<N2> setpointAnglesRad) {
    double elbowAngle = setpointAnglesRad.get(0, 0);
    double wristAngle = setpointAnglesRad.get(1, 0);
    Logger.recordOutput("Arm/Goal Arm Position", m_kinematics.forward(setpointAnglesRad));
    Logger.recordOutput("Arm/Goal Elbow Angle", elbowAngle);
    Logger.recordOutput("Arm/Goal Wrist Angle", wristAngle);
    m_setpointVisualizer.update(elbowAngle, wristAngle);
    m_elbowController.reset(m_elbowPositionRad);
    m_elbowController.setGoal(elbowAngle);
    m_wristController.reset(m_wristPositionRad);
    m_wristController.setGoal(wristAngle);
    m_profileInitialAngles = VecBuilder.fill(m_elbowPositionRad, m_wristPositionRad);
  }

  private void setSetpoint(Translation2d setpoint) {
    Optional<Vector<N2>> setpointAnglesRad = m_kinematics.inverse(setpoint);
    setpointAnglesRad.ifPresent(angles -> setSetpoint(angles));
  }

  /**
   * @param elbowDelay The percent that the wrist must travel before the elbow moves.
   * @param wristDelay The percent that the elbow must travel before the wrist moves.
   */
  private void runSetpoint(double elbowDelay, double wristDelay) {
    if (elbowDelay > 0.0 && wristDelay > 0.0)
      throw new IllegalArgumentException("Only one joint can be delayed at a time.");
    if (elbowDelay < 0.0 || wristDelay < 0.0)
      throw new IllegalArgumentException("Percent delay can't be negative.");
    double elbowGoalPosition = m_elbowController.getSetpoint().position;
    double wristGoalPosition = m_wristController.getSetpoint().position;
    double elbowGoalVelocity = m_elbowController.getSetpoint().velocity;
    double wristGoalVelocity = m_wristController.getSetpoint().velocity;
    double elbowProgress =
        Math.abs(m_elbowPositionRad - m_profileInitialAngles.get(0, 0))
            / Math.abs(m_elbowController.getGoal().position - m_profileInitialAngles.get(0, 0));
    double wristProgress =
        Math.abs(m_wristPositionRad - m_profileInitialAngles.get(1, 0))
            / Math.abs(m_wristController.getGoal().position - m_profileInitialAngles.get(1, 0));
    Logger.recordOutput("Arm/Elbow Progress", elbowProgress);
    Logger.recordOutput("Arm/Wrist Progress", wristProgress);
    double elbowFeedbackVolts = 0.0;
    double wristFeedbackVolts = 0.0;
    if (wristProgress < elbowDelay) {
      elbowGoalPosition = m_elbowPositionRad;
      elbowGoalVelocity = 0.0;
    } else
      elbowFeedbackVolts =
          m_deadbandkS.apply(m_elbowController.calculate(m_elbowPositionRad), elbowkS.get());
    if (elbowProgress < wristDelay) {
      wristGoalPosition = m_wristPositionRad;
      wristGoalVelocity = 0.0;
    } else m_deadbandkS.apply(m_wristController.calculate(m_wristPositionRad), wristkS.get());
    Vector<N2> feedforwardVolts =
        m_armModel.feedforward(
            VecBuilder.fill(elbowGoalPosition, wristGoalPosition),
            VecBuilder.fill(elbowGoalVelocity, wristGoalVelocity));
    double elbowFeedForwardVolts = feedforwardVolts.get(0, 0);
    double wristFeedForwardVolts = feedforwardVolts.get(1, 0);
    Logger.recordOutput("Arm/Elbow Feed Forward", elbowFeedForwardVolts);
    Logger.recordOutput("Arm/Wrist Feed Forward", wristFeedForwardVolts);
    m_io.setElbowVoltage(elbowFeedbackVolts + elbowFeedForwardVolts);
    m_io.setWristVoltage(wristFeedbackVolts + wristFeedForwardVolts);
  }

  public Command goToSetpoint(ArmSetpoints setpoint) {
    return this.runOnce(
            () -> {
              setSetpoint(setpoint.getTranslation());
            })
        .andThen(
            this.run(
                () -> runSetpoint(setpoint.getDelay().getFirst(), setpoint.getDelay().getSecond())))
        .withName("Set Inverse Setpoint");
  }

  public Command goToSetpoint(
      double elbowAngleRad, double wristAngleRad, double elbowDelay, double wristDelay) {
    return this.runOnce(() -> setSetpoint(VecBuilder.fill(elbowAngleRad, wristAngleRad)))
        .andThen(this.run(() -> runSetpoint(elbowDelay, wristDelay)))
        .withName("Set Forward Setpoint");
  }

  public Command idle() {
    return this.run(
            () -> {
              m_io.setElbowVoltage(0.0);
              m_io.setWristVoltage(0.0);
            })
        .withName("Idle");
  }

  public Command idleCoast() {
    return this.runOnce(() -> m_io.setBrakeMode(false))
        .andThen(
            this.run(
                () -> {
                  m_io.setElbowVoltage(0.0);
                  m_io.setWristVoltage(0.0);
                }))
        .finallyDo(() -> m_io.setBrakeMode(true))
        .withName("Idle Coast");
  }

  class ArmKinematics {

    public Translation2d forward(Vector<N2> angles) {

      double elbowAngle = angles.get(0, 0);
      double wristAngle = angles.get(1, 0);

      return new Translation2d(
          ArmModel.origin.getX()
              + ArmModel.kElbowLengthMeters * Math.cos(elbowAngle)
              + ArmModel.kWristLengthMeters * Math.cos(elbowAngle + (wristAngle - elbowAngle)),
          ArmModel.origin.getY()
              + ArmModel.kElbowLengthMeters * Math.sin(elbowAngle)
              + ArmModel.kWristLengthMeters * Math.sin(elbowAngle + (wristAngle - elbowAngle)));
    }

    public Translation2d forwardWristRelativeToElbow(Vector<N2> angles) {

      return new Translation2d(
          ArmModel.origin.getX()
              + ArmModel.kElbowLengthMeters * Math.cos(angles.get(0, 0))
              + ArmModel.kWristLengthMeters * Math.cos(angles.get(0, 0) + angles.get(1, 0)),
          ArmModel.origin.getY()
              + ArmModel.kElbowLengthMeters * Math.sin(angles.get(0, 0))
              + ArmModel.kWristLengthMeters * Math.sin(angles.get(0, 0) + angles.get(1, 0)));
    }

    // This is still broken
    public Optional<Vector<N2>> inverse(Translation2d position) {
      Translation2d relativePosition = position.minus(ArmModel.origin);

      // Flip when X is negative
      boolean isFlipped = true; // relativePosition.getX() < 0.0;
      if (isFlipped)
        relativePosition = new Translation2d(-relativePosition.getX(), relativePosition.getY());

      // Calculate angles
      double wristAngle =
          -Math.acos(
              MathUtil.clamp(
                  (Math.pow(relativePosition.getX(), 2)
                          + Math.pow(relativePosition.getY(), 2)
                          - Math.pow(ArmModel.kElbowLengthMeters, 2)
                          - Math.pow(ArmModel.kWristLengthMeters, 2))
                      / (2 * ArmModel.kElbowLengthMeters * ArmModel.kWristLengthMeters),
                  -1.0,
                  1.0));
      double elbowAngle =
          Math.atan2(relativePosition.getY(), relativePosition.getX())
              - Math.atan2(
                  (ArmModel.kWristLengthMeters * Math.sin(wristAngle)),
                  (ArmModel.kElbowLengthMeters
                      + ArmModel.kWristLengthMeters * Math.cos(wristAngle)));

      // Invert elbow angle if invalid
      Translation2d testPosition =
          forwardWristRelativeToElbow(VecBuilder.fill(elbowAngle, wristAngle))
              .minus(ArmModel.origin);
      if (testPosition.getDistance(relativePosition) > 1e-3) {
        elbowAngle += Math.PI;
      }

      // Flip angles
      if (isFlipped) {
        elbowAngle = Math.PI - elbowAngle;
        wristAngle = -wristAngle;
      }

      // Wrap angles to correct ranges
      elbowAngle = MathUtil.angleModulus(elbowAngle);
      wristAngle = MathUtil.angleModulus(wristAngle);

      // Exit if outside valid ranges for the joints
      if (elbowAngle < ArmModel.kElbowMinAngleRad
          || elbowAngle > ArmModel.kElbowMaxAngleRad
          || wristAngle < ArmModel.kWristMinAngleRad
          || wristAngle > ArmModel.kWristMaxAngleRad) {
        return Optional.empty();
      }
      return Optional.of(VecBuilder.fill(elbowAngle, wristAngle + elbowAngle));
    }
  }
}
