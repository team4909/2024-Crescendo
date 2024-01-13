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
  private double delay;

  private CANSparkMax WheelInner = new CANSparkMax(2, com.revrobotics.CANSparkLowLevel.MotorType.kBrushed);
  private CANSparkMax WheelOuter = new CANSparkMax(1, com.revrobotics.CANSparkLowLevel.MotorType.kBrushed);

  /** Creates a new KitbotShooter. */
  public KitbotShooter() {
    SmartDashboard.putNumber("ShooterDelay", 5);
    delay = SmartDashboard.getNumber("shooterDelay", 5);
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

  // public static void setDelay(double d){
  //   delay = d;
  // }

  public Command Intake() {
    return new InstantCommand(() -> {
      System.out.println("InSpeed");
      SmartDashboard.putNumber("ShooterSpeed", InSpeed);
      WheelInner.set(InSpeed);
      WheelOuter.set(InSpeed);
    }, this);
  }

  public Command WaitDelay(double seconds) {
    m_duration = seconds;
    SendableRegistry.setName(this, getName() + ": " + seconds + " seconds");
    return null;
  }

  // public Command ShooterSpeed() {
  //   return new InstantCommand(() -> {
  //     SmartDashboard.putNumber("ShooterSpeed", SmartDashboard.getNumber("ShooterSpeed", 0));
  //     WheelInner.set(SmartDashboard.getNumber("ShooterSpeed", 0));
  //     WheelOuter.set(SmartDashboard.getNumber("ShooterSpeed", 0));
  //   }, this);
  // }

  // public Command ShooterDelay() {
  //   return new SequentialCommandGroup(
  // //       WheelOuter.set(OutSpeed),
  // //       (SmartDashboard.getNumber("ShooterDelay", 0)),
  // //       WheelInner.set(OutSpeed));
  // // }

  public Command ShooterDelay() {
    return Commands.sequence(
      this.runOnce(() -> {
        WheelOuter.set(OutSpeed);
        System.out.println(SmartDashboard.getNumber("ShooterDelay", 5));
      }), 
      // Commands.waitSeconds((delay)),
      new BionicWaitCommand(() -> SmartDashboard.getNumber("ShooterDelay", 5)),
      new RepeatCommand(new InstantCommand(() -> WheelInner.set(OutSpeed))));
  }
}
