package frc.robot.arm;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.LoggedTunableNumber;
import frc.robot.Constants;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {

  public static final double kCatchWristAngleRad = 2.264 - Units.degreesToRadians(5.0);
  public static final double kSubwooferWristAngleRad = 2.083;

  private final ArmIO m_io;
  private final ArmIOInputsAutoLogged m_inputs = new ArmIOInputsAutoLogged();
  private final ArmVisualizer m_goalVisualizer, m_setpointVisualizer, m_measuredVisualizer;
  private final ArmFeedforward m_elbowFeedForward = new ArmFeedforward(0.55, 0.15, 2.31);
  private final ArmFeedforward m_wristFeedForward = new ArmFeedforward(0.4, 0.51, 0.8);

  private static final LoggedTunableNumber elbowkP = new LoggedTunableNumber("Arm/Elbow/kP");
  private static final LoggedTunableNumber elbowkD = new LoggedTunableNumber("Arm/Elbow/kD");
  private static final LoggedTunableNumber elbowkS = new LoggedTunableNumber("Arm/Elbow/kS");
  private static final LoggedTunableNumber elbowkV = new LoggedTunableNumber("Arm/Elbow/kV");
  private static final LoggedTunableNumber elbowkG = new LoggedTunableNumber("Arm/Elbow/kG");
  private static final LoggedTunableNumber elbowCruiseVelocityRadPerSec =
      new LoggedTunableNumber("Arm/Elbow/CruiseVelocity");
  private static final LoggedTunableNumber elbowMaxAccelerationRadPerSecSq =
      new LoggedTunableNumber("Arm/Elbow/MaxAcceleration");
  private static final LoggedTunableNumber wristkP = new LoggedTunableNumber("Arm/Wrist/kP");
  private static final LoggedTunableNumber wristkD = new LoggedTunableNumber("Arm/Wrist/kD");
  private static final LoggedTunableNumber wristkS = new LoggedTunableNumber("Arm/Wrist/kS");
  private static final LoggedTunableNumber wristkV = new LoggedTunableNumber("Arm/Wrist/kV");
  private static final LoggedTunableNumber wristkG = new LoggedTunableNumber("Arm/Wrist/kG");
  private static final LoggedTunableNumber wristCruiseVelocityRadPerSec =
      new LoggedTunableNumber("Arm/Wrist/CruiseVelocity");
  private static final LoggedTunableNumber wristMaxAccelerationRadPerSecSq =
      new LoggedTunableNumber("Arm/Wrist/MaxAcceleration");
  private static final LoggedTunableNumber wristAimTuningAngle =
      new LoggedTunableNumber("Arm/Wrist/TuningAngle");
  private static final LoggedTunableNumber elbowAimTuningAngle =
      new LoggedTunableNumber("Arm/Elbow/TuningAngle");

  private Vector<N2> m_profileInitialAngles;
  public Supplier<Pose3d> wristPoseSupplier;
  private final SysIdRoutine m_sysIdRoutineElbow, m_sysIdRoutineWrist;

  static {
    switch (Constants.kCurrentMode) {
      case kReal:
        elbowkP.initDefault(2.1);
        elbowkD.initDefault(0.8);
        elbowCruiseVelocityRadPerSec.initDefault(5.0);
        elbowMaxAccelerationRadPerSecSq.initDefault(8.0);
        wristkP.initDefault(3.3);
        wristkD.initDefault(0);
        wristCruiseVelocityRadPerSec.initDefault(4.0);
        wristMaxAccelerationRadPerSecSq.initDefault(8.0);
        elbowAimTuningAngle.initDefault(Math.toDegrees(ArmSetpoints.kStowed.elbowAngle));
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
        wristAimTuningAngle.initDefault(Math.toDegrees(ArmSetpoints.kStowed.wristAngle));
        break;
      default:
        break;
    }
  }

  public Arm(ArmIO io) {
    m_io = io;
    m_goalVisualizer = new ArmVisualizer("ArmGoal", 2, Color.kGreen);
    m_setpointVisualizer = new ArmVisualizer("ArmSetpoint", 4, Color.kGreen);
    m_measuredVisualizer = new ArmVisualizer("ArmMeasured", 6, Color.kDarkGreen);
    wristPoseSupplier = () -> m_measuredVisualizer.getWristPose();
    m_sysIdRoutineElbow =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.25).per(Seconds.of(1.0)),
                Volts.of(2.0),
                null,
                state -> Logger.recordOutput("Arm/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                voltage -> m_io.setElbowVoltage(voltage.in(Volts)), null, this));
    m_sysIdRoutineWrist =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.25).per(Seconds.of(1.0)),
                Volts.of(2.0),
                null,
                state -> Logger.recordOutput("Arm/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                voltage -> m_io.setWristVoltage(voltage.in(Volts)), null, this));
    setDefaultCommand(idle());
  }

  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("ArmInputs", m_inputs);

    if (DriverStation.isDisabled()) {
      m_io.setElbowVoltage(0.0);
      m_io.setWristVoltage(0.0);
    }

    // #region Update controllers wßhen tunables change.
    if (elbowkP.hasChanged(hashCode())
        || elbowkD.hasChanged(hashCode())
        || wristkP.hasChanged(hashCode())
        || wristkD.hasChanged(hashCode()))
      m_io.configPD(elbowkP.get(), elbowkD.get(), wristkP.get(), wristkD.get());

    if (elbowkS.hasChanged(hashCode())
        || elbowkV.hasChanged(hashCode())
        || elbowkG.hasChanged(hashCode())
        || wristkS.hasChanged(hashCode())
        || wristkV.hasChanged(hashCode())
        || wristkG.hasChanged(hashCode()))
      m_io.configFF(
          elbowkS.get(), elbowkV.get(), elbowkG.get(), wristkS.get(), wristkV.get(), wristkG.get());

    if (elbowCruiseVelocityRadPerSec.hasChanged(hashCode())
        || elbowMaxAccelerationRadPerSecSq.hasChanged(hashCode())
        || wristCruiseVelocityRadPerSec.hasChanged(hashCode())
        || wristMaxAccelerationRadPerSecSq.hasChanged(hashCode()))
      m_io.configLimits(
          elbowCruiseVelocityRadPerSec.get(),
          elbowMaxAccelerationRadPerSecSq.get(),
          wristCruiseVelocityRadPerSec.get(),
          wristMaxAccelerationRadPerSecSq.get());

    // #endregion

    m_measuredVisualizer.update(
        Units.rotationsToRadians(m_inputs.elbowPositionRot),
        Units.rotationsToRadians(m_inputs.wristPositionRot));
    Logger.recordOutput(
        "Arm/Current Command",
        getCurrentCommand() == null ? "Null" : getCurrentCommand().getName());
  }

  /**
   * @param elbowDelay The percent that the wrist must travel before the elbow moves.
   * @param wristDelay The percent that the elbow must travel before the wrist moves.
   */
  private void runSetpoint(
      double elbowAngleDeg, double wristAngleDeg, double elbowDelay, double wristDelay) {
    if (elbowDelay > 0.0 && wristDelay > 0.0)
      throw new IllegalArgumentException("Only one joint can be delayed at a time.");
    if (elbowDelay < 0.0 || wristDelay < 0.0)
      throw new IllegalArgumentException("Percent delay can't be negative.");
    Logger.recordOutput("Arm/Goal Elbow Angle", elbowAngleDeg);
    Logger.recordOutput("Arm/Goal Wrist Angle", wristAngleDeg);
    m_goalVisualizer.update(
        Units.degreesToRadians(elbowAngleDeg), Units.degreesToRadians(wristAngleDeg));
    m_setpointVisualizer.update(
        Units.rotationsToRadians(m_inputs.elbowPositionSetpointRot),
        Units.rotationsToRadians(m_inputs.wristPositionSetpointRot));
    double elbowProgress =
        Math.abs(m_inputs.elbowPositionRot - m_profileInitialAngles.get(0, 0))
            / Math.abs(Units.degreesToRotations(elbowAngleDeg) - m_profileInitialAngles.get(0, 0));
    double wristProgress =
        Math.abs(m_inputs.wristPositionRot - m_profileInitialAngles.get(1, 0))
            / Math.abs(Units.degreesToRotations(wristAngleDeg) - m_profileInitialAngles.get(1, 0));
    Logger.recordOutput("Arm/Elbow Progress", elbowProgress);
    Logger.recordOutput("Arm/Wrist Progress", wristProgress);
    if (wristProgress < elbowDelay) m_io.setElbowRotations(m_inputs.elbowPositionRot);
    else m_io.setElbowRotations(Units.degreesToRotations(elbowAngleDeg));
    if (elbowProgress < wristDelay) m_io.setWristRotations(m_inputs.wristPositionRot);
    else m_io.setWristRotations(Units.degreesToRotations(wristAngleDeg));
  }

  public Command storeInitialAngles =
      this.runOnce(
          () ->
              m_profileInitialAngles =
                  VecBuilder.fill(m_inputs.elbowPositionRot, m_inputs.wristPositionRot));

  public Command aimElbow(double elbowAngleRad) {
    return storeInitialAngles
        .andThen(
            this.run(() -> runSetpoint(elbowAngleRad, ArmSetpoints.kStowed.wristAngle, 0.0, 0.0)))
        .withName("Aim Elbow");
  }

  public Command aimWrist(double wristAngleRad) {
    return storeInitialAngles
        .andThen(
            this.run(() -> runSetpoint(ArmSetpoints.kStowed.elbowAngle, wristAngleRad, 0.0, 0.0)))
        .withName("Aim Wrist");
  }

  public Command aimElbowForTuning() {
    return Commands.defer(() -> aimElbow(Math.toRadians(elbowAimTuningAngle.get())), Set.of(this));
  }

  public Command aimWristForTuning() {
    return Commands.defer(() -> aimWrist(Math.toRadians(wristAimTuningAngle.get())), Set.of(this));
  }

  public Trigger readyToShoot() {
    return new Trigger(() -> getJointsAtGoal()).debounce(0.2, DebounceType.kBoth);
  }

  public Command goToSetpoint(ArmSetpoints setpoint) {
    return storeInitialAngles
        .andThen(
            this.run(
                () ->
                    runSetpoint(
                        setpoint.elbowAngle,
                        setpoint.wristAngle,
                        setpoint.elbowDelay,
                        setpoint.wristDelay)))
        .withName("Set Inverse Setpoint");
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
        .ignoringDisable(true)
        .withName("Idle Coast");
  }

  public Command sysIdElbowQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineElbow.quasistatic(direction);
  }

  public Command sysIdElbowDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineElbow.dynamic(direction);
  }

  public Command sysIdWristQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineWrist.quasistatic(direction);
  }

  public Command sysIdWristDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineWrist.dynamic(direction);
  }

  private boolean getJointsAtGoal() {
    return m_elbowController.atGoal() && m_wristController.atGoal();
  }

  public static enum ArmSetpoints {
    kStowed(-0.548, 2.485, 0.15, 0.0),
    kAmp(1.49 + 0.0873, -2.307, 0.0, 0.0),
    kClimb(1.633, -2.371, 0.0, 0.0);

    public final double elbowAngle;
    public final double wristAngle;
    public final double elbowDelay;
    public final double wristDelay;

    private ArmSetpoints(
        double elbowAngle, double wristAngle, double elbowDelay, double wristDelay) {
      this.elbowAngle = elbowAngle;
      this.wristAngle = wristAngle;
      this.elbowDelay = elbowDelay;
      this.wristDelay = wristDelay;
    }
  }
}
