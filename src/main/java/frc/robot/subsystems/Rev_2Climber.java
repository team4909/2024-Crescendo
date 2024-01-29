// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rev_2Climber extends SubsystemBase {
  /** Creates a new Rev_2Climber. */

  private final double DefaultUpSpeed = -1;
  private final double DefaultdownSpeed = 1;
  private final double StopSpeed = 0;

  private CANSparkMax LeftHook = new CANSparkMax(10, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
  private CANSparkMax RightHook = new CANSparkMax(11, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

  public Rev_2Climber() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    LeftHook.setIdleMode(IdleMode.kBrake);
    RightHook.setIdleMode(IdleMode.kBrake);

    SmartDashboard.putNumber("DefaultUpSpeed", DefaultUpSpeed);
    SmartDashboard.putNumber("DefaultdownSpeed", DefaultdownSpeed);
  }

  public Command Stop() {
    return new InstantCommand(() -> {
      LeftHook.set(StopSpeed);
      RightHook.set(StopSpeed);
    }, this);
  }

  public Command up() {
    return new InstantCommand(() -> {
      double LeftHookUp = SmartDashboard.getNumber("Up Speed", DefaultUpSpeed);
      LeftHook.set(LeftHookUp);

      double RightHookUp = SmartDashboard.getNumber("Up Speed", DefaultUpSpeed);
      RightHook.set(RightHookUp);
    }, this);
  }

  public Command down() {
    return new InstantCommand(() -> {
      double LeftHookDown = SmartDashboard.getNumber("Down Speed", DefaultUpSpeed);
      LeftHook.set(LeftHookDown);

      double RightHookDown = SmartDashboard.getNumber("Down Speed", DefaultUpSpeed);
      RightHook.set(RightHookDown);
    }, this);
  }

}
