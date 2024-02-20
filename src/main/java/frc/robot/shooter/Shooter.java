// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.BionicWaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final double InSpeed = -1;
  private final double OutSpeed = 1;
  private final double StopSpeed = 0;
  private double defaultDelay = .3;

  private TalonFX feeder = new TalonFX(19, "CANivore2");
  private TalonFX shooterTop = new TalonFX(20, "CANivore2");
  private TalonFX shooterBottom = new TalonFX(21, "CANivore2");

  private DigitalInput m_noteSensor = new DigitalInput(0);

  public boolean hasNote() {
    return m_noteSensor.get();
  }

  /** Creates a new Rev_1Shooter. */
  public Shooter() {
    SmartDashboard.putNumber("ShooterDelay", defaultDelay);

    TalonFXConfiguration brakeMode = new TalonFXConfiguration();
    brakeMode.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    shooterTop.getConfigurator().apply(brakeMode);
    shooterBottom.getConfigurator().apply(brakeMode);
    feeder.getConfigurator().apply(brakeMode);
    setDefaultCommand(PullBack().until(() -> !hasNote()));
  }

  @Override
  public void periodic() {
    Logger.recordOutput(
        "Shooter Command", getCurrentCommand() == null ? "" : getCurrentCommand().getName());
  }

  public Command Shoot() {
    return ShooterOn().repeatedly();
  }

  public Command ShooterOn() {
    return new RunCommand(
        () -> {
          shooterTop.set(OutSpeed);
          shooterBottom.set(OutSpeed);
        },
        this);
  }

  public Command ShooterOff() {
    return new InstantCommand(
        () -> {
          shooterTop.set(StopSpeed);
          shooterBottom.set(StopSpeed);
          feeder.set(StopSpeed);
        },
        this);
  }

  public Command StopRepeatedly() {
    return new InstantCommand(
            () -> {
              SmartDashboard.putNumber("ShooterSpeed", StopSpeed);

              shooterTop.set(StopSpeed);
              shooterBottom.set(StopSpeed);
              feeder.set(StopSpeed);
            },
            this)
        .repeatedly();
  }

  public Command Stop() {
    return new RunCommand(
            () -> {
              SmartDashboard.putNumber("ShooterSpeed", StopSpeed);

              shooterTop.set(StopSpeed);
              shooterBottom.set(StopSpeed);
              feeder.set(StopSpeed);
            },
            this)
        .withName("Stop");
  }

  public Command FeederOn() {
    return new RunCommand(
            () -> {
              feeder.set(InSpeed);
            },
            this)
        .withName("FeederOn");
  }

  public Command FeederOn(double speed) {
    return new InstantCommand(
        () -> {
          feeder.set(speed);
        },
        this);
  }

  public Command Catch() {
    return new RunCommand(
        () -> {
          feeder.set(-0.15);
          shooterTop.set(-OutSpeed);
          shooterBottom.set(-OutSpeed);
        },
        this);
  }

  public Command PullBack() {
    return new RunCommand(() -> feeder.set(0.3), this);
  }

  public Command Feeder() {
    return new InstantCommand(
            () -> {
              feeder.set(-OutSpeed);
            },
            this)
        .repeatedly()
        .withTimeout(1)
        .finallyDo(() -> StopRepeatedly());
  }

  public Command FeederOff() {
    return new InstantCommand(
        () -> {
          feeder.set(0);
        });
  }

  public double getCurrent() {
    return feeder.getTorqueCurrent().getValue();
  }

  public Command FeederOut() {
    return new InstantCommand(
        () -> {
          feeder.set(OutSpeed);
        },
        this);
  }

  public Command ShooterDelay() {
    return Commands.sequence(
        ShooterOn(),
        new BionicWaitCommand(() -> SmartDashboard.getNumber("ShooterDelay", defaultDelay)),
        FeederOn());
  }
}
