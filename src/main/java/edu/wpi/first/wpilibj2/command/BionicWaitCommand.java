// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.wpilibj2.command;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Timer;

/**
 * A command that does nothing but takes a specified amount of time to finish.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class BionicWaitCommand extends Command {
  /** The timer used for waiting. */
  protected Timer m_timer = new Timer();

  // private final double m_duration;
  private final DoubleSupplier m_delaySeconds;

  /**
   * Creates a new WaitCommand. This command will do nothing, and end after the specified duration.
   *
   * @param seconds the time to wait, in seconds
   */
  @SuppressWarnings("this-escape")
  public BionicWaitCommand(double seconds) {
    // m_duration = seconds;
    m_delaySeconds = () -> seconds;
    SendableRegistry.setName(this, getName() + ": " + seconds + " seconds");
  }

  public BionicWaitCommand(DoubleSupplier delaySeconds) {
    m_delaySeconds = delaySeconds;
    SendableRegistry.setName(this, getName() + ": DYNAMIC seconds");
  }

  @Override
  public void initialize() {
    m_timer.restart();
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_delaySeconds.getAsDouble());
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("duration", () -> m_delaySeconds.getAsDouble(), null);
  }
}
