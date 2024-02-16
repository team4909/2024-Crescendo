// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  // private final double Speed = 1;
  private final double StopSpeed = 0;
  private double defaultFrontRollerSpeed = -.8;
  private double defaultBackRollerSpeed = -.8;
  private double defaultFrontRollerSpeedSpit = .8;
  private double defaultBackRollerSpeedSpit = .8;

  private CANSparkMax topRoller = new CANSparkMax(5, CANSparkMax.MotorType.kBrushless);
  private CANSparkMax bottomRoller = new CANSparkMax(6, CANSparkMax.MotorType.kBrushless);

  public Intake() {
    SmartDashboard.putNumber("FrontRollerSpeed", defaultFrontRollerSpeed);
    SmartDashboard.putNumber("BackRollerSpeed", defaultBackRollerSpeed);
    SmartDashboard.putNumber("FrontRollerSpeedSpit", defaultFrontRollerSpeed);
    SmartDashboard.putNumber("BackRollerSpeedSpit", defaultBackRollerSpeed);

    topRoller.setIdleMode(IdleMode.kBrake);
    bottomRoller.setIdleMode(IdleMode.kBrake);

    // frontRoller.getFault(FaultID.)
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command Stop() {
    return new InstantCommand(
        () -> {
          topRoller.set(StopSpeed);
          bottomRoller.set(StopSpeed);
        },
        this);
  }

  public Command Spit() {
    return new RunCommand(
        () -> {
          double FrontRollerSpeedSpit =
              SmartDashboard.getNumber("FrontRollerSpeedSpit", defaultFrontRollerSpeedSpit);
          topRoller.set(defaultFrontRollerSpeedSpit);

          double BackRollerSpeedSpit =
              SmartDashboard.getNumber("BackRollerSpeedSpit", defaultBackRollerSpeedSpit);
          bottomRoller.set(defaultBackRollerSpeedSpit);
        },
        this);
  }

  public Command intake(boolean spit) {
    int direction = spit ? -1 : 1;
    return new RunCommand(
        () -> {
          topRoller.set(direction * defaultFrontRollerSpeed);
          bottomRoller.set(direction * defaultBackRollerSpeed);
        },
        this);
  }
}
