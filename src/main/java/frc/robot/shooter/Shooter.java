// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.BionicWaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private final double InSpeed = -.4;
  private final double OutSpeed = .4;
  private final double StopSpeed = 0;
  private double defaultDelay = .3;

  private TalonFX feeder = new TalonFX(19, "CANivore2");
  private TalonFX shooterTop = new TalonFX(17, "CANivore2");
  private TalonFX shooterBottom = new TalonFX(18, "CANivore2");

  /** Creates a new Rev_1Shooter. */
  public Shooter() {
    SmartDashboard.putNumber("ShooterDelay", defaultDelay);
    shooterTop.getConfigurator().apply(new TalonFXConfiguration());
    shooterBottom.getConfigurator().apply(new TalonFXConfiguration());
    feeder.getConfigurator().apply(new TalonFXConfiguration());
  }

  @Override
  public void periodic() {}

  public Command Shoot() {
    return new InstantCommand(
            () -> {
              SmartDashboard.putNumber("ShooterSpeed", OutSpeed);

              shooterTop.set(OutSpeed);
              shooterBottom.set(OutSpeed);
              feeder.set(InSpeed);
            },
            this)
        .repeatedly();
  }

  public Command Stop() {
    return new InstantCommand(
        () -> {
          SmartDashboard.putNumber("ShooterSpeed", StopSpeed);

          shooterTop.set(StopSpeed);
          shooterBottom.set(StopSpeed);
          feeder.set(StopSpeed);
        },
        this);
  }

  public Command Intake() {
    return new InstantCommand(
        () -> {
          System.out.println("InSpeed");
          SmartDashboard.putNumber("ShooterSpeed", InSpeed);

          feeder.set(InSpeed);
        },
        this);
  }

  public Command ShooterDelay() {
    return Commands.sequence(
        this.runOnce(
            () -> {
              shooterTop.set(OutSpeed);
              shooterBottom.set(OutSpeed);
              // System.out.println(SmartDashboard.getNumber("ShooterDelay", defaultDelay));
            }),
        new BionicWaitCommand(() -> SmartDashboard.getNumber("ShooterDelay", defaultDelay)),
        new RepeatCommand(
            new InstantCommand(
                () -> {
                  feeder.set(OutSpeed);
                })));
  }

  public Command Feeder() {
    return new InstantCommand(
            () -> {
              feeder.set(OutSpeed);
            },
            this)
        .repeatedly();
  }

  public double getCurrent(){
    return feeder.getTorqueCurrent().getValue();

  }
}
