package frc.robot.arm;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.LoggedTunableNumber;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {

  public static final double kCatchWristAngleRad = 2.264 - Units.degreesToRadians(5.0);
  public static final double kSubwooferWristAngleRad = 2.083;

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
                Volts.of(1.0).per(Seconds.of(1.0)),
                Volts.of(10.0),
                null,
                state -> SignalLogger.writeString("ElbowSysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                voltage -> m_io.setElbowCurrent(voltage.in(Volts)), null, this));
    m_sysIdRoutineWrist =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1.0).per(Seconds.of(1.0)),
                Volts.of(10.0),
                null,
                state -> SignalLogger.writeString("WristSysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                voltage -> m_io.setWristCurrent(voltage.in(Volts)), null, this));
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
      double elbowAngleRad, double wristAngleRad, double elbowDelay, double wristDelay) {
    if (elbowDelay > 0.0 && wristDelay > 0.0)
      throw new IllegalArgumentException("Only one joint can be delayed at a time.");
    if (elbowDelay < 0.0 || wristDelay < 0.0)
      throw new IllegalArgumentException("Percent delay can't be negative.");
    Logger.recordOutput("Arm/Goal Elbow Angle", elbowAngleRad);
    Logger.recordOutput("Arm/Goal Wrist Angle", wristAngleRad);
    m_goalVisualizer.update(elbowAngleRad, wristAngleRad);
    m_setpointVisualizer.update(
        Units.rotationsToRadians(m_inputs.elbowPositionSetpointRot),
        Units.rotationsToRadians(m_inputs.wristPositionSetpointRot));
    double elbowProgress =
        Math.abs(m_inputs.elbowPositionRot - m_initialAngles.get(0, 0))
            / Math.abs(Units.degreesToRotations(elbowAngleRad) - m_initialAngles.get(0, 0));
    double wristProgress =
        Math.abs(m_inputs.wristPositionRot - m_initialAngles.get(1, 0))
            / Math.abs(Units.degreesToRotations(wristAngleRad) - m_initialAngles.get(1, 0));
    Logger.recordOutput("Arm/Elbow Progress", elbowProgress);
    Logger.recordOutput("Arm/Wrist Progress", wristProgress);
    if (wristProgress < elbowDelay) m_io.setElbowRotations(m_inputs.elbowPositionRot);
    else m_io.setElbowRotations(Units.radiansToRotations(elbowAngleRad));
    if (elbowProgress < wristDelay) m_io.setWristRotations(m_inputs.wristPositionRot);
    else m_io.setWristRotations(Units.radiansToRotations(wristAngleRad));
  }

  public Command storeInitialAngles() {
    return this.runOnce(
        () ->
            m_initialAngles =
                VecBuilder.fill(m_inputs.elbowPositionRot, m_inputs.wristPositionRot));
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
    return this.run(
            () -> {
              m_io.setWristRotations(Units.radiansToRotations(ArmSetpoints.kStowed.wristAngle));
              m_io.setElbowVoltage(MathUtil.applyDeadband(driveEffort.getAsDouble(), 0.1) * 3.0);
            })
        .finallyDo(() -> holdSetpoint().schedule());
  }

  public Command aimWristForTuning(DoubleSupplier driveEffort) {
    return this.run(
        () -> {
          m_io.setElbowRotations(Units.radiansToRotations(ArmSetpoints.kStowed.elbowAngle));
          m_io.setWristVoltage(MathUtil.applyDeadband(driveEffort.getAsDouble(), 0.1) * 1.0);
        });
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
        .withName("Hold Setpoint");
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
        .withName("Set Inverse Setpoint");
  }

  public Command idle() {
    return this.run(() -> m_io.stop()).withName("Idle");
  }

  public Command idleCoast() {
    return this.runOnce(() -> m_io.setBrakeMode(false))
        .andThen(this.run(() -> m_io.stop()))
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
