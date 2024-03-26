package frc.robot.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  public static final double kWinchReduction = 1.0;
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
          m_io.setLeftVoltage(0.0);
          m_io.setRightVoltage(0.0);
        });
  }

  public Command windWinch() {
    return this.run(
        () -> {
          m_io.setLeftVoltage(12.0);
          m_io.setRightVoltage(12.0);
        });
  }

  public Command unwindWinch() {
    return this.run(
        () -> {
          m_io.setLeftVoltage(-4.0);
          m_io.setRightVoltage(-4.0);
        });
  }
}
