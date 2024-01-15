// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.BionicWaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class KitbotShooter extends SubsystemBase {

  private final double InSpeed = 0.45;
  private final double OutSpeed = -1;
  private final double StopSpeed = 0;
  private double m_duration;
  private double defaultDelay = .3;

  private CANSparkMax WheelInner = new CANSparkMax(2, com.revrobotics.CANSparkLowLevel.MotorType.kBrushed);
  private CANSparkMax WheelOuter = new CANSparkMax(1, com.revrobotics.CANSparkLowLevel.MotorType.kBrushed);

  /** Creates a new KitbotShooter. */
  public KitbotShooter() {
    SmartDashboard.putNumber("ShooterDelay", defaultDelay);
    // delay = SmartDashboard.getNumber("shooterDelay", 5);
  }

  @Override
  public void periodic() {

  }

  public Command Shoot() {
    return new InstantCommand(() -> {
      System.out.println("OutSpeed");
      SmartDashboard.putNumber("ShooterSpeed", OutSpeed);
      WheelInner.set(OutSpeed);
      WheelOuter.set(OutSpeed);
    }, this);
  }

  public Command Stop() {
    return new InstantCommand(() -> {
      // System.out.println("StopSpeed");
      SmartDashboard.putNumber("ShooterSpeed", StopSpeed);
      WheelInner.set(StopSpeed);
      WheelOuter.set(StopSpeed);
    }, this);
  }

  public Command Intake() {
    return new InstantCommand(() -> {
      System.out.println("InSpeed");
      SmartDashboard.putNumber("ShooterSpeed", InSpeed);
      WheelInner.set(InSpeed);
      WheelOuter.set(InSpeed);
    }, this);
  }

  public Command ShooterDelay() {
    return Commands.sequence(
      this.runOnce(() -> {
        WheelOuter.set(OutSpeed);
        System.out.println(SmartDashboard.getNumber("ShooterDelay", defaultDelay));
      }), 
      new BionicWaitCommand(() -> SmartDashboard.getNumber("ShooterDelay", defaultDelay)),
      new RepeatCommand(new InstantCommand(() -> WheelInner.set(OutSpeed))));
  }
}
