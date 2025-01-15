package frc.robot.climber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  public static final double kWinchReduction = 20.0;
  private final ClimberIO m_io;
  private final ClimberIOInputsAutoLogged m_inputs = new ClimberIOInputsAutoLogged();
  private final double kTrapSetpointRot = 10.5;
  private final PIDController m_leftController, m_rightController;
  public final Trigger atTrapSetpoint;

  public Climber(ClimberIO climberIO) {
    m_io = climberIO;
    m_leftController = new PIDController(120.0, 0.0, 0.0);
    m_rightController = new PIDController(120.0, 0.0, 0.0);
    m_leftController.setTolerance(0.05);
    m_rightController.setTolerance(0.05);
    atTrapSetpoint =
        new Trigger(() -> m_leftController.atSetpoint() && m_rightController.atSetpoint());
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

  public Command goToTrapLimit() {
    return Commands.parallel(
            this.run(
                () -> {
                  if (m_inputs.leftWinchPositionRot <= kTrapSetpointRot) m_io.setLeftVoltage(12.0);
                  else m_io.setLeftVoltage(0.0);
                  if (m_inputs.leftWinchPositionRot <= kTrapSetpointRot) m_io.setRightVoltage(12.0);
                  else m_io.setRightVoltage(0.0);
                }))
        .until(
            () ->
                m_inputs.leftWinchPositionRot > kTrapSetpointRot
                    && m_inputs.leftWinchPositionRot > kTrapSetpointRot);
  }

  public Command setTrapSetpoint() {
    return this.run(
            () -> {
              m_io.setLeftVoltage(
                  m_leftController.calculate(m_inputs.leftWinchPositionRot, kTrapSetpointRot));
              m_io.setRightVoltage(
                  m_rightController.calculate(m_inputs.rightWinchPositionRot, kTrapSetpointRot));
            })
        .finallyDo(
            () -> {
              m_leftController.reset();
              m_rightController.reset();
            })
        .withName("Trap Setpoint (Climber)");
  }
}
