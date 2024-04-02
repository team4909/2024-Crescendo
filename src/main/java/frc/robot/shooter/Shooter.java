package frc.robot.shooter;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  public static final double kShooterStepUp = Constants.kIsViper ? 1.0 : 1.0;
  public static final double kFarShotVelocityRpm = 5800.0;
  private final double kTrapShot = 400.0;
  private final double kAmpShot = 5000.0;
  private final double kReadyToShootToleranceRps = 3.0;

  // Denominator for gains here are in rotations
  public static final double topRollerkS = 0.15945;
  public static final double topRollerkV = 0.11596;
  public static final double topRollerkA = 0.013645;
  public static final double bottomRollerkS = 0.11589;
  public static final double bottomRollerkV = 0.11618;
  public static final double bottomRollerkA = 0.015157;

  public static final double topRollerkP = 0.176;
  public static final double bottomRollerkP = 0.176;

  private final SysIdRoutine m_sysIdRoutine;
  private final ShooterIO m_io;
  private final ShooterIOInputsAutoLogged m_inputs = new ShooterIOInputsAutoLogged();

  public final Trigger readyToShoot =
      new Trigger(this::getRollersAtSetpoint).debounce(0.1, DebounceType.kBoth);

  private double m_lastSetpoint;

  public Shooter(ShooterIO io) {
    m_io = io;

    m_sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                state -> SignalLogger.writeString("Shooter/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                voltage -> {
                  m_io.setTopRollerVoltage(voltage.in(Volts));
                  m_io.setBottomRollerVoltage(voltage.in(Volts));
                },
                null,
                this));
    setDefaultCommand(idle());
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("ShooterInputs", m_inputs);
  }

  private void setRollersSetpointRpm(double velocityRpm) {
    double goalVelocityRps = velocityRpm / 60.0;
    Logger.recordOutput("Shooter/Goal Roller RPS", goalVelocityRps);
    m_lastSetpoint = goalVelocityRps;
    m_io.setTopRollerVelocity(goalVelocityRps);
    m_io.setBottomRollerVelocity(goalVelocityRps);
  }

  @AutoLogOutput(key = "Shooter/RollersAtSetpoint")
  private boolean getRollersAtSetpoint() {
    return MathUtil.isNear(
            m_lastSetpoint, m_inputs.bottomRollerVelocityRps, kReadyToShootToleranceRps)
        && MathUtil.isNear(
            m_lastSetpoint, m_inputs.topRollerVelocityRps, kReadyToShootToleranceRps);
  }

  public Command sysId() {
    return Commands.sequence(
        m_sysIdRoutine.quasistatic(Direction.kForward),
        Commands.waitSeconds(10.0),
        m_sysIdRoutine.quasistatic(Direction.kReverse),
        Commands.waitSeconds(10.0),
        m_sysIdRoutine.dynamic(Direction.kForward),
        Commands.waitSeconds(10.0),
        m_sysIdRoutine.dynamic(Direction.kReverse));
  }

  public Command idle() {
    return this.run(
            () -> {
              m_io.setTopRollerVoltage(0.0);
              m_io.setBottomRollerVoltage(0.0);
            })
        .withName("Idle (Shooter)");
  }

  public Command stop() {
    return this.runOnce(m_io::stopRollers);
  }

  public Command runShooter() {
    return this.run(() -> setRollersSetpointRpm(kFarShotVelocityRpm))
        .finallyDo(() -> Logger.recordOutput("Shooter/Goal Roller RPS", 0.0))
        .withName("Run Shooter (Shooter)");
  }

  public Command trapShot() {
    return this.run(() -> setRollersSetpointRpm(kTrapShot))
        .finallyDo(() -> Logger.recordOutput("Shooter/Goal Roller RPS", 0.0))
        .withName("Trap Shot (Shooter)");
  }

  public Command stashShot() {
    return this.run(
            () -> {
              m_io.setTopRollerVoltage(12.0);
              m_io.setBottomRollerVoltage(12.0);
            })
        .withName("Stash Shot (Shooter)");
  }

  public Command ampShot() {
    return this.run(() -> setRollersSetpointRpm(kAmpShot));
  }

  public Command catchNote() {
    return this.run(
            () -> {
              m_io.setTopRollerVoltage(-9.0);
              m_io.setBottomRollerVoltage(-9.0);
            })
        .withName("Catch (Shooter)");
  }

  public Command spit() {
    return this.run(
            () -> {
              m_io.setTopRollerVoltage(-6.0);
              m_io.setBottomRollerVoltage(-6.0);
            })
        .withName("Spit (Shooter)");
  }
}
