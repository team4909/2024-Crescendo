// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignal.SignalMeasurement;
// import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class Arm extends SubsystemBase {
  private double m_j1Ratio;
  private double m_j2Ratio;
  private double m_MPRatio;

  private TalonFX m_lJoint1 = new TalonFX(15, "CANivore2");
  private TalonFX m_lJoint2 = new TalonFX(16, "CANivore2");
  private TalonFX m_rJoint1 = new TalonFX(17, "CANivore2");
  private TalonFX m_rJoint2 = new TalonFX(18, "CANivore2");

  private double m_angleOffset = 32d;

  private StatusSignal<Double> signal;

  // Change jerk if needed

  final DynamicMotionMagicVoltage m_j1Request = new DynamicMotionMagicVoltage(0, 24, 12, 0);
  final DynamicMotionMagicVoltage m_j2Request = new DynamicMotionMagicVoltage(0, 10, 20, 20);

  final DynamicMotionMagicVoltage m_goDownRequest = new DynamicMotionMagicVoltage(0, 10, 20, 20);

  /** Creates a new Rev_2Arm. */
  public Arm() {
    m_MPRatio = 15d;
    m_j1Ratio = 36d / 22d * m_MPRatio;
    m_j2Ratio = 36d / 22d * m_MPRatio;

    // ArmMotorFirstP.setPosition(High);
    m_rJoint1.setControl(new Follower(m_lJoint1.getDeviceID(), true));
    m_rJoint2.setControl(new Follower(m_lJoint2.getDeviceID(), true));

    m_lJoint1.setPosition(m_angleOffset);
    m_lJoint2.setPosition(0);

    TalonFXConfiguration armConfigs = new TalonFXConfiguration();
    armConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    armConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;


    m_lJoint1.getConfigurator().apply(armConfigs);
    m_lJoint2.getConfigurator().apply(armConfigs);

    var l_joint1Configs = new Slot0Configs();
    l_joint1Configs.kS = 0.5;
    l_joint1Configs.kV = 0.5;
    l_joint1Configs.kG = 9.8;
    l_joint1Configs.kP = 0.5;
    l_joint1Configs.kI = 0;
    l_joint1Configs.kD = 0;
    l_joint1Configs.GravityType = GravityTypeValue.Arm_Cosine;

    // apply gains, 50 ms total timeout
    m_lJoint1.getConfigurator().apply(l_joint1Configs, 0.050);

    var l_joint2Configs = new Slot0Configs();
    l_joint2Configs.kS = 0.15;
    l_joint2Configs.kV = 0.25;
    l_joint2Configs.kA = 0.05;
    l_joint2Configs.kP = 0.65;
    l_joint2Configs.kI = 0;
    l_joint2Configs.kD = 0;

    // apply gains, 50 ms total timeout
    m_lJoint2.getConfigurator().apply(l_joint2Configs, 0.050);
  }

  @Override
  public void periodic() {
    signal = m_lJoint1.getPosition().refresh();
    System.out.println(signal.getValueAsDouble());
  }

  private Command goToDeg(
      TalonFX joint, DynamicMotionMagicVoltage request, double gearRatio, double degree) {
    return new InstantCommand(
        () -> {
          joint.setControl(request.withPosition((degree * gearRatio) / (360d)));
        },
        this);
  }

  public Command goToDeg(double j1Degrees, double j2Degrees) {
    return new SequentialCommandGroup(
        goToDeg(m_lJoint1, m_j1Request, m_j1Ratio, -j1Degrees),
        goToDeg(m_lJoint2, m_j2Request, m_j2Ratio, j2Degrees));
  }

  public Command goToDegSeq(double j1ParDeg, double j2ParDeg, double j2SeqDeg) {
    return new SequentialCommandGroup(
        goToDeg(j1ParDeg, j2ParDeg),
        new WaitUntilCommand(
            () -> {
              return Math.abs(m_lJoint1.getPosition().getValue() - (-j1ParDeg * m_j1Ratio) / (360d))
                  <= 20;
            }),
        goToDeg(m_lJoint2, m_j2Request, m_j2Ratio, j2SeqDeg));
  }

  public Command goDown() {
    return new SequentialCommandGroup(
        goToDeg(m_lJoint2, m_j2Request, m_j2Ratio, 0),
        new WaitUntilCommand(
            () -> {
              return Math.abs(m_lJoint2.getPosition().getValue() - (0 * m_j2Ratio) / (360d)) <= 5;
            }),
        new SequentialCommandGroup(
            goToDeg(m_lJoint1, m_goDownRequest, m_j1Ratio, 0),
            goToDeg(m_lJoint2, m_goDownRequest, m_j2Ratio, 0)));
  }

  public Command goDownAuto() {
    return new SequentialCommandGroup(
        goToDeg(m_lJoint1, m_j1Request, m_j1Ratio, -10),
        new WaitUntilCommand(
            () -> {
              return Math.abs(m_lJoint1.getPosition().getValue() - (-10 * m_j1Ratio) / (360d)) <= 5;
            }),
        new SequentialCommandGroup(
            goToDeg(m_lJoint1, m_goDownRequest, m_j1Ratio, 0),
            goToDeg(m_lJoint2, m_goDownRequest, m_j2Ratio, 0)));
  }
}
