// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.BionicWaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rev_1Shooter extends SubsystemBase {

  private final double InSpeed = 0.45;
  private final double OutSpeed = -1;
  private final double StopSpeed = 0;
  private double defaultDelay = .3;

  private CANSparkMax Feeder = new CANSparkMax(4, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
  private CANSparkMax Shooter = new CANSparkMax(3, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
  private CANSparkMax Feeder1 = new CANSparkMax(2, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
  private CANSparkMax Shooter2 = new CANSparkMax(1, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

  /** Creates a new Rev_1Shooter. */
  public Rev_1Shooter() {
    SmartDashboard.putNumber("ShooterDelay", defaultDelay);
  }

  @Override
  public void periodic() {

  }

  public Command Shoot() {
    return new InstantCommand(() -> {
      System.out.println("OutSpeed");
      SmartDashboard.putNumber("ShooterSpeed", OutSpeed);
      Feeder.set(OutSpeed);
      Shooter.set(OutSpeed);
      Feeder1.set(OutSpeed);
      Shooter2.set(OutSpeed);
    }, this);
  }

  public Command Stop() {
    return new InstantCommand(() -> {
      // System.out.println("StopSpeed");
      SmartDashboard.putNumber("ShooterSpeed", StopSpeed);
      Feeder.set(StopSpeed);
      Shooter.set(StopSpeed);
      Feeder1.set(OutSpeed);
      Shooter2.set(OutSpeed);
    }, this);
  }

  public Command Intake() {
    return new InstantCommand(() -> {
      System.out.println("InSpeed");
      SmartDashboard.putNumber("ShooterSpeed", InSpeed);
      Feeder.set(InSpeed);
      Shooter.set(InSpeed);
      Feeder1.set(OutSpeed);
      Shooter2.set(OutSpeed);
    }, this);
  }

  public Command ShooterDelay() {
    return Commands.sequence(
        this.runOnce(() -> {
          Shooter.set(OutSpeed);
          // System.out.println(SmartDashboard.getNumber("ShooterDelay", defaultDelay));
        }),
        new BionicWaitCommand(() -> SmartDashboard.getNumber("ShooterDelay", defaultDelay)),
        new RepeatCommand(new InstantCommand(() -> Feeder.set(OutSpeed))));
  }
}
