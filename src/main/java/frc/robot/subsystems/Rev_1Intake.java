// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rev_1Intake extends SubsystemBase {
  private final double Speed = 1;
  private final double StopSpeed = 0;
  private final double SpitSpeed = -1;

  private CANSparkMax frontRoller = new CANSparkMax(5, CANSparkMax.MotorType.kBrushless);
  private CANSparkMax backRoller = new CANSparkMax(6, CANSparkMax.MotorType.kBrushless);

  public Rev_1Intake() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command Intake() {
    return new InstantCommand(() -> {
      frontRoller.set(Speed);
      backRoller.set(Speed);
    }, this);
  }

  public Command Stop() {
    return new InstantCommand(() -> {
      frontRoller.set(StopSpeed);
      backRoller.set(StopSpeed);
    }, this);
  }

  public Command Spit() {
    return new InstantCommand(() -> {
      frontRoller.set(SpitSpeed);
      backRoller.set(SpitSpeed);
    }, this);
  }

}
