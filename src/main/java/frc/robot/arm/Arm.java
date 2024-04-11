package frc.robot.arm;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {

  public static final double kCatchWristAngleRad = 2.264 - Units.degreesToRadians(5.0);
  public static final double kSubwooferWristAngleRad = 2.083;
  public static final double kDeployGizmoAngleRad = Units.degreesToRadians(80.0);
  private final double kJointTolerenceDegrees = 2.0;

  private final ArmIO m_io;
  private final ArmIOInputsAutoLogged m_inputs = new ArmIOInputsAutoLogged();
  private final ArmVisualizer m_goalVisualizer, m_setpointVisualizer, m_measuredVisualizer;

  private static final LoggedTunableNumber elbowCruiseVelocityRadPerSec =
      new LoggedTunableNumber("Arm/Elbow/CruiseVelocity");
  private static final LoggedTunableNumber elbowMaxAccelerationRadPerSecSq =
      new LoggedTunableNumber("Arm/Elbow/MaxAcceleration");
  private static final LoggedTunableNumber wristCruiseVelocityRadPerSec =
      new LoggedTunableNumber("Arm/Wrist/CruiseVelocity");
  private static final LoggedTunableNumber wristMaxAccelerationRadPerSecSq =
      new LoggedTunableNumber("Arm/Wrist/MaxAcceleration");

  private Vector<N2> m_initialAngles;
  private double m_lastElbowSetpoint;
  private double m_lastWristSetpoint;
  public Trigger atGoal = new Trigger(this::jointsAtGoal).debounce(0.1, DebounceType.kBoth);
  public Supplier<Pose3d> wristPoseSupplier;
  private final SysIdRoutine m_sysIdRoutineElbow, m_sysIdRoutineWrist;

  static {
    switch (Constants.kCurrentMode) {
      case kReal:
        elbowCruiseVelocityRadPerSec.initDefault(5.0);
        elbowMaxAccelerationRadPerSecSq.initDefault(8.0);
        wristCruiseVelocityRadPerSec.initDefault(4.0);
        wristMaxAccelerationRadPerSecSq.initDefault(8.0);
        break;
      case kSim:
        elbowCruiseVelocityRadPerSec.initDefault(3.0);
        elbowMaxAccelerationRadPerSecSq.initDefault(2.0);
        wristCruiseVelocityRadPerSec.initDefault(5.0);
        wristMaxAccelerationRadPerSecSq.initDefault(2.0);
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
                state -> SignalLogger.writeString("ElbowSysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                voltage -> m_io.setElbowVoltage(voltage.in(Volts)), null, this));
    m_sysIdRoutineWrist =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.25).per(Seconds.of(1.0)),
                Volts.of(2.0),
                null,
                state -> SignalLogger.writeString("WristSysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                voltage -> m_io.setWristVoltage(voltage.in(Volts)), null, this));
    setDefaultCommand(idle());
  }

  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("ArmInputs", m_inputs);

    if (DriverStation.isDisabled()) m_io.stop();

    if (elbowCruiseVelocityRadPerSec.hasChanged(hashCode())
        || elbowMaxAccelerationRadPerSecSq.hasChanged(hashCode())
        || wristCruiseVelocityRadPerSec.hasChanged(hashCode())
        || wristMaxAccelerationRadPerSecSq.hasChanged(hashCode()))
      m_io.configLimits(
          elbowCruiseVelocityRadPerSec.get(),
          elbowMaxAccelerationRadPerSecSq.get(),
          wristCruiseVelocityRadPerSec.get(),
          wristMaxAccelerationRadPerSecSq.get());

    Logger.recordOutput(
        "Arm/Current Elbow Angle Degrees", Units.rotationsToDegrees(m_inputs.elbowPositionRot));
    Logger.recordOutput(
        "Arm/Current Wrist Angle Degrees", Units.rotationsToDegrees(m_inputs.wristPositionRot));
    m_measuredVisualizer.update(
        Units.rotationsToRadians(m_inputs.elbowPositionRot),
        Units.rotationsToRadians(m_inputs.wristPositionRot));
    m_setpointVisualizer.update(
        Units.rotationsToRadians(m_inputs.elbowPositionSetpointRot),
        Units.rotationsToRadians(m_inputs.wristPositionSetpointRot));
  }

  /**
   * @param elbowDelay The percent that the wrist must travel before the elbow moves.
   * @param wristDelay The percent that the elbow must travel before the wrist moves.
   */
  private void runSetpoint(
      double elbowAngleRad, double wristAngleRad, double elbowDelay, double wristDelay) {
    if (elbowDelay > 0.0 && wristDelay > 0.0)
      throw new IllegalArgumentException("Only one joint can be delayed at a time.");
    if (elbowDelay < 0.0 || wristDelay < 0.0)
      throw new IllegalArgumentException("Percent delay can't be negative.");
    m_lastElbowSetpoint = elbowAngleRad;
    m_lastWristSetpoint = wristAngleRad;
    Logger.recordOutput("Arm/Goal Elbow Angle", Units.radiansToDegrees(elbowAngleRad));
    Logger.recordOutput("Arm/Goal Wrist Angle", Units.radiansToDegrees(wristAngleRad));
    m_goalVisualizer.update(elbowAngleRad, wristAngleRad);
    double elbowProgress =
        Math.abs(m_inputs.elbowPositionRot - m_initialAngles.get(0, 0))
            / Math.abs(Units.degreesToRotations(elbowAngleRad) - m_initialAngles.get(0, 0));
    double wristProgress =
        Math.abs(m_inputs.wristPositionRot - m_initialAngles.get(1, 0))
            / Math.abs(Units.degreesToRotations(wristAngleRad) - m_initialAngles.get(1, 0));
    Logger.recordOutput("Arm/Elbow Progress", elbowProgress);
    Logger.recordOutput("Arm/Wrist Progress", wristProgress);

    // if (wristProgress < elbowDelay) m_io.setElbowRotations(m_inputs.elbowPositionRot);
    m_io.setElbowRotations(Units.radiansToRotations(elbowAngleRad));
    // if (elbowProgress < wristDelay) m_io.setWristRotations(m_inputs.wristPositionRot);
    m_io.setWristRotations(Units.radiansToRotations(wristAngleRad));
  }

  @AutoLogOutput(key = "Arm/JointsAtGoal")
  private boolean jointsAtGoal() {
    return (MathUtil.isNear(
            m_lastElbowSetpoint,
            Units.rotationsToRadians(m_inputs.elbowPositionRot),
            Units.degreesToRadians(kJointTolerenceDegrees))
        && MathUtil.isNear(
            m_lastWristSetpoint,
            Units.rotationsToRadians(m_inputs.wristPositionRot),
            Units.degreesToRadians(kJointTolerenceDegrees)));
  }

  public Command storeInitialAngles() {
    return this.runOnce(
        () ->
            m_initialAngles =
                VecBuilder.fill(m_inputs.elbowPositionRot, m_inputs.wristPositionRot));
  }

  // Joint Index: 0 = elbow, 1 = wrist
  public Command aim(Supplier<Joint> jointIndexSupplier, Supplier<Rotation2d> angleSupplier) {
    return storeInitialAngles()
        .andThen(
            this.run(
                () -> {
                  if (jointIndexSupplier.get() == Joint.kElbow)
                    runSetpoint(
                        angleSupplier.get().getRadians(),
                        ArmSetpoints.kStowed.wristAngle,
                        0.0,
                        0.0);
                  else if (jointIndexSupplier.get() == Joint.kWrist)
                    runSetpoint(
                        ArmSetpoints.kStowed.elbowAngle,
                        angleSupplier.get().getRadians(),
                        0.0,
                        0.0);
                  else throw new IllegalArgumentException("Invalid joint index.");
                }))
        .withName("Aim (Arm)");
  }

  public Command aimElbow(double elbowAngleRad) {
    return storeInitialAngles()
        .andThen(
            this.run(() -> runSetpoint(elbowAngleRad, ArmSetpoints.kStowed.wristAngle, 0.0, 0.0)))
        .withName("Aim Elbow");
  }

  public Command aimWrist(double wristAngleRad) {
    return storeInitialAngles()
        .andThen(
            this.run(() -> runSetpoint(ArmSetpoints.kStowed.elbowAngle, wristAngleRad, 0.0, 0.0)))
        .withName("Aim Wrist");
  }

  public Command aimElbowForTuning(DoubleSupplier driveEffort) {
    var state =
        new Object() {
          double currentPos = ArmSetpoints.kStowed.elbowAngle;
        };
    return Commands.run(
            () -> state.currentPos += MathUtil.applyDeadband(driveEffort.getAsDouble(), 0.1) * 0.05)
        .alongWith(aim(() -> Joint.kElbow, () -> Rotation2d.fromRadians(state.currentPos)))
        .withName("Aim Elbow for Tuning");
  }

  public Command aimWristForTuning(DoubleSupplier driveEffort) {
    var state =
        new Object() {
          double currentPos = ArmSetpoints.kStowed.wristAngle;
        };
    return Commands.run(
            () -> state.currentPos += MathUtil.applyDeadband(driveEffort.getAsDouble(), 0.1) * 0.05)
        .alongWith(aim(() -> Joint.kWrist, () -> Rotation2d.fromRadians(state.currentPos)))
        .withName("Aim Wrist For Tuning");
  }

  public Command holdSetpoint() {
    return storeInitialAngles()
        .andThen(
            this.run(
                () ->
                    runSetpoint(
                        Units.rotationsToRadians(m_initialAngles.get(0)),
                        Units.rotationsToRadians(m_initialAngles.get(1)),
                        0.0,
                        0.0)))
        .withName("Hold Setpoint (Arm)");
  }

  public Command goToSetpoint(ArmSetpoints setpoint) {
    return storeInitialAngles()
        .andThen(
            this.run(
                () ->
                    runSetpoint(
                        setpoint.elbowAngle,
                        setpoint.wristAngle,
                        setpoint.elbowDelay,
                        setpoint.wristDelay)))
        .withName("Go To Setpoint (Arm)");
  }

  public Command idle() {
    return this.run(m_io::stop).withName("Idle (Arm)");
  }

  public Command stop() {
    return this.runOnce(m_io::stop);
  }

  public Command idleCoast() {
    return this.runOnce(() -> m_io.setBrakeMode(false))
        .andThen(this.run(() -> m_io.stop()))
        .finallyDo(() -> m_io.setBrakeMode(true))
        .ignoringDisable(true)
        .withName("Idle Coast (Arm)");
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

  public Command elbowStaticCharacterization() {
    return new ArmStaticCharacterization(
        this, m_io::setElbowCurrent, () -> Units.rotationsToRadians(m_inputs.elbowVelocityRps));
  }

  public Command wristStaticCharacterization() {
    return new ArmStaticCharacterization(
        this, m_io::setWristCurrent, () -> Units.rotationsToRadians(m_inputs.wristVelocityRps));
  }

  public enum ArmSetpoints {
    kStowed(Units.degreesToRadians(-31), Units.degreesToRadians(142), 0.15, 0.0),
    kAmp(1.49 + 0.0873 + Units.degreesToRadians(3.0), Units.degreesToRadians(228), 0.0, 0.0),
    kClimb(1.633, -2.371, 0.0, 0.0),
    kTrap(Units.degreesToRadians(53.0), Units.degreesToRadians(80.0), 0.0, 0.0),
    kStash(Units.degreesToRadians(-31), 2.083, 0.0, 0.0);

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

  public enum Joint {
    kElbow,
    kWrist
  }
}
