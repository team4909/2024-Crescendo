package frc.robot.arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.LoggedTunableNumber;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class ArmStaticCharacterization extends Command {
  private static final LoggedTunableNumber currentRampFactor =
      new LoggedTunableNumber("StaticCharacterization/CurrentRampPerSec", 1.0);
  private static final LoggedTunableNumber minVelocity =
      new LoggedTunableNumber("StaticCharacterization/MinStaticVelocity", 0.1);

  private final DoubleConsumer m_inputConsumer;
  private final DoubleSupplier m_velocitySupplier;
  private final Timer m_timer = new Timer();
  private double m_currentInput = 0.0;

  public ArmStaticCharacterization(
      Arm arm, DoubleConsumer characterizationInputConsumer, DoubleSupplier velocitySupplier) {
    m_inputConsumer = characterizationInputConsumer;
    this.m_velocitySupplier = velocitySupplier;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    m_timer.restart();
  }

  @Override
  public void execute() {
    m_currentInput = m_timer.get() * currentRampFactor.get();
    m_inputConsumer.accept(m_currentInput);
  }

  @Override
  public boolean isFinished() {
    return m_velocitySupplier.getAsDouble() >= minVelocity.get();
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Static Characterization output: " + m_currentInput + " amps");
    m_inputConsumer.accept(0);
  }
}
