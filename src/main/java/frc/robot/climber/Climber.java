package frc.robot.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final ClimberIO m_climberIO;
  private final ClimberIOInputsAutoLogged m_inputs = new ClimberIOInputsAutoLogged();

  public Climber(ClimberIO climberIO) {
    m_climberIO = climberIO;

    setDefaultCommand(idle());
  }

  @Override
  public void periodic() {
    m_climberIO.updateInputs(m_inputs);
    Logger.processInputs("ClimberInputs", m_inputs);
  }

  public Command idle() {
    return this.run(
        () -> {
          m_climberIO.setLeftDutyCycle(0.0);
          m_climberIO.setRightDutyCycle(0.0);
        });
  }

  public Command winchDown() {
    return this.run(
        () -> {
          m_climberIO.setLeftDutyCycle(1.0);
          m_climberIO.setRightDutyCycle(1.0);
        });
  }

  public Command release() {
    return this.run(
        () -> {
          m_climberIO.setLeftDutyCycle(-0.25);
          m_climberIO.setRightDutyCycle(-0.25);
        });
  }
}
