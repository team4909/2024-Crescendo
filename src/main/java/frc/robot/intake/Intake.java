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
  private final double defaultFrontRollerSpeed = .8;
  private final double defaultBackRollerSpeed = .8;
  private final double defaultFrontRollerSpeedSpit = -.8;
  private final double defaultBackRollerSpeedSpit = -.8;
  private final double CenteringBagSpeed = 0.5;
  private final double SpitCenteringBagSpeed = -0.5;

  private CANSparkMax topRoller = new CANSparkMax(5, CANSparkMax.MotorType.kBrushless);
  private CANSparkMax bottomRoller = new CANSparkMax(6, CANSparkMax.MotorType.kBrushless);
  private CANSparkMax centeringBagMotors = new CANSparkMax(8, CANSparkMax.MotorType.kBrushed);

  public Intake() {
    SmartDashboard.putNumber("FrontRollerSpeed", defaultFrontRollerSpeed);
    SmartDashboard.putNumber("BackRollerSpeed", defaultBackRollerSpeed);
    SmartDashboard.putNumber("FrontRollerSpeedSpit", defaultFrontRollerSpeed);
    SmartDashboard.putNumber("BackRollerSpeedSpit", defaultBackRollerSpeed);

    topRoller.setIdleMode(IdleMode.kBrake);
    bottomRoller.setIdleMode(IdleMode.kBrake);

    centeringBagMotors.setSmartCurrentLimit(15); // leaving this to high so Bag motors dont burn out

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
          centeringBagMotors.set(StopSpeed);
        },
        this);
  }

  public Command Spit() {
    return new InstantCommand(
        () -> {
          topRoller.set(defaultFrontRollerSpeedSpit);
          bottomRoller.set(defaultBackRollerSpeedSpit);
          centeringBagMotors.set(SpitCenteringBagSpeed);
        },
        this);
  }

  public Command intake(boolean spit) {
    int direction = spit ? -1 : 1;
    return new RunCommand(
        () -> {
          topRoller.set(direction * defaultFrontRollerSpeed);
          bottomRoller.set(direction * defaultBackRollerSpeed);
          centeringBagMotors.set(direction * CenteringBagSpeed);
        },
        this);
  }

  public Command release() {
    return new RunCommand(
        () -> {
          topRoller.set(0.4);
          bottomRoller.set(0.4);
          centeringBagMotors.set(CenteringBagSpeed);
        },
        this);
  }
}
