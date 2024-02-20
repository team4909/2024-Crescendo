package frc.robot.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO m_io;
  private final IntakeIOInputsAutoLogged m_inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    m_io = io;
    setDefaultCommand(null);
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("IntakeInputs", m_inputs);
  }

  public Command idle() {
    return this.run(
        () -> {
          m_io.setTopRollerDutyCycle(0.0);
          m_io.setBottomRollerDutyCycle(0.0);
          m_io.setCenteringMotorsDutyCycle(0.0);
        });
  }

  public Command spit() {
    return this.run(
        () -> {
          m_io.setTopRollerDutyCycle(-0.8);
          m_io.setBottomRollerDutyCycle(-0.8);
          m_io.setCenteringMotorsDutyCycle(-0.5);
        });
  }

  public Command intake() {
    return this.run(
        () -> {
          m_io.setTopRollerDutyCycle(0.8);
          m_io.setBottomRollerDutyCycle(0.8);
          m_io.setCenteringMotorsDutyCycle(0.5);
        });
  }

  public Command feed() {
    return this.run(
        () -> {
          m_io.setTopRollerDutyCycle(0.4);
          m_io.setBottomRollerDutyCycle(0.4);
          m_io.setCenteringMotorsDutyCycle(0.5);
        });
  }
}
