// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ARM;

// import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.Idle;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.ARM.Rev_2Arm.LowMotorConstants;
import frc.robot.subsystems.ARM.Rev_2Arm.UpperMotorConstants;

public class Rev_2Arm extends SubsystemBase {

  public static final class LowMotorConstants {
    public static final double IDLE = 0;
    public static final double LOW = 0;
    public static final double MID = 10;
    public static final double HIGH = 0;
    public static final double RETRACTED = 0;
    // public static final double LOWKV = 0;
    // public static final double LOWKP = 0;
    // public static final double LOWKI = 0;
    // public static final double LOWKD = 0;
  }

  public static final class UpperMotorConstants {
    public static final double IDLE = 0;
    public static final double LOW = 0;
    public static final double MID = 10;
    public static final double HIGH = 0;
    public static final double RETRACTED = 0;
    // public static final double UPKV = 0;
    // public static final double UPKP = 0;
    // public static final double UPKI = 0;
    // public static final double UPKD = 0;
  }

  private TalonFX lowMotor = new TalonFX(8, "CANivore1");
  private TalonFX upperMotor = new TalonFX(6, "CANivore1");
  // private TalonFX lowMotorFollower = new TalonFX(14);
  // private TalonFX upperMotorFollower = new TalonFX(16);

  final PositionVoltage m_request = new PositionVoltage(10).withSlot(0);

  private final DutyCycleOut m_lMotor = new DutyCycleOut(1.0);
  private final DutyCycleOut m_rMotor = new DutyCycleOut(1.0);

  /** Creates a new Rev_2Arm. */
  public Rev_2Arm() {
    // ArmMotorFirstP.setPosition(High);
    // lowMotorFollower.setControl(new Follower(lowMotor.getDeviceID(), false));
    // lowMotorFollower.follow(lowMotor);
    // upperMotorFollower.setControl(new Follower(lowMotor.getDeviceID(), false));

    // lowMotor.configFactoryDefault();
    // upperMotor.configFactoryDefault();
    // lowMotorFollower.configFactoryDefault();
    // upperMotorFollower.configFactoryDefault();
    // lowMotor.setSmartCurrentLimit(40);
    // upperMotor.setSmartCurrentLimit(40);

    var lowMotorSlot0Configs = new Slot0Configs();
    lowMotorSlot0Configs.kV = 0.;
    lowMotorSlot0Configs.kP = 0.1;
    lowMotorSlot0Configs.kI = 0;
    lowMotorSlot0Configs.kD = 0;

    // apply gains, 50 ms total timeout
    lowMotor.getConfigurator().apply(lowMotorSlot0Configs, 0.050);

    var slot1Configs = new Slot0Configs();
    slot1Configs.kV = 0.12;
    slot1Configs.kP = 0.11;
    slot1Configs.kI = 0.5;
    slot1Configs.kD = 0.001;

    // apply gains, 50 ms total timeout
    upperMotor.getConfigurator().apply(slot1Configs, 0.050);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // public Command idle() {
  // return new InstantCommand(() -> {
  // lowMotor.setControl(m_request.withPosition(LowMotorConstants.IDLE));
  // upperMotor.setControl(m_request.withPosition(UpperMotorConstants.IDLE));
  // }, this);
  // }

  public Command low() {
    return new InstantCommand(() -> {
      System.out.println("LOW");
      // lowMotor.setPosition(LowMotorConstants.LOW);
      // upperMotor.setPosition(UpperMotorConstants.LOW);
      lowMotor.setControl(m_request.withPosition(1));
      // upperMotor.setControl(new DutyCycleOut(1));
      upperMotor.setControl(m_request.withPosition(10));
    }, this);
  }

  public Command stop() {
    return new InstantCommand(() -> {
      // lowMotor.setPosition(LowMotorConstants.LOW);
      // upperMotor.setPosition(UpperMotorConstants.LOW);
      lowMotor.setControl(new DutyCycleOut(0));
      // upperMotor.setControl(new DutyCycleOut(1));
      upperMotor.setControl(new DutyCycleOut(0));
    }, this);
  }

  // public Command mid() {
  // return new InstantCommand(() -> {
  // System.out.println("MID COMMAND");
  // // set position to 10 rotations
  // lowMotor.setControl(m_request.withPosition(LowMotorConstants.MID));
  // upperMotor.setControl(m_request.withPosition(UpperMotorConstants.MID));
  // }, this);
  // }

  // public Command high() {
  // return new InstantCommand(() -> {
  // lowMotor.setControl(m_request.withPosition(LowMotorConstants.HIGH));
  // upperMotor.setControl(m_request.withPosition(UpperMotorConstants.HIGH));
  // }, this);
  // }

  // public Command retracted() {
  // return new InstantCommand(() -> {
  // lowMotor.setControl(m_request.withPosition(LowMotorConstants.RETRACTED));
  // upperMotor.setControl(m_request.withPosition(UpperMotorConstants.RETRACTED));
  // }, this);
  // }

}
