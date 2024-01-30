// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rev_2Climber extends SubsystemBase {
  /** Creates a new Rev_2Climber. */
  private CANSparkMax LeftHook = new CANSparkMax(10, CANSparkMax.MotorType.kBrushless);

  private RelativeEncoder leftEnc = LeftHook.getEncoder();
  // private CANSparkMax RightHook = new CANSparkMax(11,
  // com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

  public Rev_2Climber() {
    // RightHook.follow(LeftHook);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Elevator Enc Value", leftEnc.getPosition());
  }

  public Command Stop() {
    return new InstantCommand(
        () -> {
          LeftHook.set(0);
        });
  }

  public Command raiseElevator() {
    PIDController leftController = new PIDController(0.05, 0, 0);
    leftController.setTolerance(10);
    return new PIDCommand(
        leftController,
        () -> leftEnc.getPosition(),
        () -> 2,
        (leftSpeed) -> LeftHook.set(leftSpeed),
        this);
  }

  public Command lowerElevator() {
    PIDController leftController = new PIDController(0.05, 0, 0);
    leftController.setTolerance(1);
    return new PIDCommand(
        leftController,
        () -> leftEnc.getPosition(),
        () -> 0,
        (leftSpeed) -> LeftHook.set(leftSpeed),
        this);
  }
}
