package frc.robot.shooter;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final double kFarShotVelocityRpm = 5000.0;
  private final double kSpitVelocityRpm = -500.0;
  private final double kIdleVelocityRpm = 0.0;

  private final PIDController m_topRollerController, m_bottomRollerController;
  private final SimpleMotorFeedforward m_topRollerFeedforward, m_bottomRollerFeedforward;
  private final SysIdRoutine m_sysIdRoutine;

  private final ShooterIO m_io;
  private final ShooterIOInputsAutoLogged m_inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) {
    m_io = io;

    m_topRollerFeedforward = new SimpleMotorFeedforward(0.18039, 0.11968);
    m_topRollerController = new PIDController(0.13085, 0.0, 0.0);

    m_bottomRollerFeedforward = new SimpleMotorFeedforward(0.19936, 0.12041);
    m_bottomRollerController = new PIDController(0.11992, 0.0, 0.0);

    m_topRollerController.setTolerance(1.0);
    m_bottomRollerController.setTolerance(1.0);
    m_sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Arm/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
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
    Logger.recordOutput(
        "Shooter/Command", getCurrentCommand() == null ? "" : getCurrentCommand().getName());
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

  private boolean getShooterRollersAtSetpoint() {
    return m_bottomRollerController.atSetpoint() && m_topRollerController.atSetpoint();
  }

  public Trigger readyToShoot() {
    return new Trigger(() -> getShooterRollersAtSetpoint()).debounce(0.25);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  public Command idle() {
    return this.run(
            () -> {
              m_io.setRollerDutyCycle(0.0);
              // setRollersSetpointRpm(kIdleVelocityRpm);
            })
        .withName("Idle");
  }

  public Command runShooter() {
    return this.run(
        () -> {
          m_io.setRollerDutyCycle(1.0);
          // setRollersSetpointRpm(kFarShotVelocityRpm);
        });
  }

  // public Command spit() {
  //   return this.run(
  //       () -> {
  //         m_io.setRollersRPS(-kSpitVelocityRpm);
  //       });
  // }

  // public Command disableShooter() {
  //   return this.run(
  //       () -> {
  //         m_io.setRollersRPS(0.0);
  //       });
  // }

  public Command catchNote() {
    return this.run(
            () -> m_io.setRollerDutyCycle(-8)
            // setRollersSetpointRpm(-kFarShotVelocityRpm)
            )
        .withName("Catch Note");
  }

  public Command shootWithFeederDelay() {
    return this.run(() -> setRollersSetpointRpm(kFarShotVelocityRpm))
        .until(() -> getShooterRollersAtSetpoint())
        .andThen(runShooter());
  }
}
