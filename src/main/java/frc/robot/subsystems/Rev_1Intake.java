// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rev_1Intake extends SubsystemBase {
  // private final double Speed = 1;
  private final double StopSpeed = 0;
  private double defaultFrontRollerSpeed = 1;
  private double defaultBackRollerSpeed = 1;
  private double defaultFrontRollerSpeedSpit = -1;
  private double defaultBackRollerSpeedSpit = -1;

  private CANSparkMax frontRoller = new CANSparkMax(5, CANSparkMax.MotorType.kBrushless);
  private CANSparkMax backRoller = new CANSparkMax(6, CANSparkMax.MotorType.kBrushless);

  public Rev_1Intake() {
    SmartDashboard.putNumber("FrontRollerSpeed", defaultFrontRollerSpeed);
    SmartDashboard.putNumber("BackRollerSpeed", defaultBackRollerSpeed);
    SmartDashboard.putNumber("FrontRollerSpeedSpit", defaultFrontRollerSpeed);
    SmartDashboard.putNumber("BackRollerSpeedSpit", defaultBackRollerSpeed);

    frontRoller.setIdleMode(IdleMode.kBrake);
    backRoller.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command Stop() {
    return new InstantCommand(() -> {
      frontRoller.set(StopSpeed);
      backRoller.set(StopSpeed);
    }, this);
  }

  public Command Spit() {
    return new InstantCommand(() -> {
      double FrontRollerSpeedSpit = SmartDashboard.getNumber("FrontRollerSpeedSpit", defaultFrontRollerSpeedSpit);
      frontRoller.set(FrontRollerSpeedSpit);

      double BackRollerSpeedSpit = SmartDashboard.getNumber("BackRollerSpeedSpit", defaultBackRollerSpeedSpit);
      backRoller.set(BackRollerSpeedSpit);
    }, this);
  }

  public Command Intake() {
    return new InstantCommand(() -> {
      double FrontRollerSpeed = SmartDashboard.getNumber("FrontRollerSpeed", defaultFrontRollerSpeed);
      frontRoller.set(FrontRollerSpeed);

      double BackRollerSpeed = SmartDashboard.getNumber("BackRollerSpeed", defaultBackRollerSpeed);
      backRoller.set(BackRollerSpeed);
    }, this);
  }

}
