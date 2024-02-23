package frc.robot.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final ClimberIO m_io;
  private final ClimberIOInputsAutoLogged m_inputs = new ClimberIOInputsAutoLogged();

  public Climber(ClimberIO climberIO) {
    m_io = climberIO;
    setDefaultCommand(idle());
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("ClimberInputs", m_inputs);
  }

  public Command idle() {
    return this.run(
        () -> {
          m_io.setLeftDutyCycle(0.0);
          m_io.setRightDutyCycle(0.0);
        });
  }

  public Command windWinch() {
    return this.runOnce(() -> m_io.enableBrakeMode(true))
        .andThen(
            this.run(
                () -> {
                  m_io.setLeftDutyCycle(1.0);
                  m_io.setRightDutyCycle(1.0);
                }))
        .finallyDo(() -> m_io.enableBrakeMode(false));
  }

  public Command unwindWinch() {
    return this.run(
        () -> {
          m_io.setLeftDutyCycle(-0.25);
          m_io.setRightDutyCycle(-0.25);
        });
  }
}
