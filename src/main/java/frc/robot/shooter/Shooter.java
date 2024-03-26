package frc.robot.shooter;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final double kFarShotVelocityRpm = 5600.0;
  private final double kAmpshot = 5000.0;
  private final double kReadyToShootToleranceRps = 3.0;

  // Denominator for gains here are in rotations
  public static final double topRollerkS = 0.18039;
  public static final double topRollerkV = 0.11968;
  public static final double topRollerkA = 0.0089044;
  public static final double bottomRollerkS = 0.19936;
  public static final double bottomRollerkV = 0.12041;
  public static final double bottomRollerkA = 0.0071461;

  private final double topRollerkP = 0.13085;
  private final double bottomRollerkP = 0.11992;

  private final PIDController m_topRollerController, m_bottomRollerController;
  private final SimpleMotorFeedforward m_topRollerFeedforward, m_bottomRollerFeedforward;
  private final SysIdRoutine m_sysIdRoutine;

  private final ShooterIO m_io;
  private final ShooterIOInputsAutoLogged m_inputs = new ShooterIOInputsAutoLogged();

  public final Trigger readyToShoot =
      new Trigger(() -> getRollersAtSetpoint()).debounce(0.1, DebounceType.kBoth);

  public Shooter(ShooterIO io) {
    m_io = io;

    m_topRollerFeedforward = new SimpleMotorFeedforward(topRollerkS, topRollerkV, topRollerkA);
    m_topRollerController = new PIDController(topRollerkP, 0.0, 0.0);
    m_bottomRollerFeedforward =
        new SimpleMotorFeedforward(bottomRollerkS, bottomRollerkV, bottomRollerkA);
    m_bottomRollerController = new PIDController(bottomRollerkP, 0.0, 0.0);

    m_topRollerController.setTolerance(kReadyToShootToleranceRps);
    m_bottomRollerController.setTolerance(kReadyToShootToleranceRps);
    m_sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                state -> Logger.recordOutput("Shooter/SysIdState", state.toString())),
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
    double topRollerFeedforwardOutput = m_topRollerFeedforward.calculate(goalVelocityRps);
    double bottomRollerFeedforwardOutput = m_bottomRollerFeedforward.calculate(goalVelocityRps);
    double topRollerFeedbackOutput =
        m_topRollerController.calculate(m_inputs.topRollerVelocityRps, goalVelocityRps);
    double bottomRollerFeedbackOutput =
        m_bottomRollerController.calculate(m_inputs.bottomRollerVelocityRps, goalVelocityRps);
    m_io.setTopRollerVoltage(topRollerFeedforwardOutput + topRollerFeedbackOutput);
    m_io.setBottomRollerVoltage(bottomRollerFeedforwardOutput + bottomRollerFeedbackOutput);
  }

  @AutoLogOutput(key = "Shooter/RollersAtSetpoint")
  private boolean getRollersAtSetpoint() {
    return m_bottomRollerController.atSetpoint() && m_topRollerController.atSetpoint();
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
    return this.run(
            () -> {
              setRollersSetpointRpm(kFarShotVelocityRpm);
            })
        .finallyDo(
            () -> {
              Logger.recordOutput("Shooter/Goal Roller RPS", 0.0);
              m_topRollerController.reset();
              m_bottomRollerController.reset();
            })
        .withName("Spin Up (Shooter)");
  }

  public Command trap() {
    return this.run(
            () -> {
              setRollersSetpointRpm(2800);
            })
        .finallyDo(
            () -> {
              Logger.recordOutput("Shooter/Goal Roller RPS", 0.0);
              m_topRollerController.reset();
              m_bottomRollerController.reset();
            })
        .withName("Spin Up (Shooter)");
  }

  public Command ampShot() {
    return this.run(() -> setRollersSetpointRpm(kAmpshot));
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

  public Command shootWithFeederDelay() {
    return this.run(() -> setRollersSetpointRpm(kFarShotVelocityRpm))
        .until(() -> getRollersAtSetpoint())
        .andThen(runShooter());
  }
}
