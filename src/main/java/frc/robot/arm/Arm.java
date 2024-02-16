// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.arm;

// import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
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

  private TalonFX m_lJoint1 = new TalonFX(13, "CANivore2");
  private TalonFX m_lJoint2 = new TalonFX(15, "CANivore2");
  private TalonFX m_rJoint1 = new TalonFX(14, "CANivore2");
  private TalonFX m_rJoint2 = new TalonFX(16, "CANivore2");

  // Change jerk if needed
  // final DynamicMotionMagicVoltage m_j1Request = new
  // DynamicMotionMagicVoltage(0, 1000, 200,
  // 1000);
  // final DynamicMotionMagicVoltage m_j2Request = new
  // DynamicMotionMagicVoltage(0, 1000, 200, 250);
  final DynamicMotionMagicVoltage m_j1Request = new DynamicMotionMagicVoltage(0, 1000, 200, 200);
  final DynamicMotionMagicVoltage m_j2Request = new DynamicMotionMagicVoltage(0, 1000, 200, 200);
  final DynamicMotionMagicVoltage m_goDownRequest = new DynamicMotionMagicVoltage(0, 10, 20, 20);

  /** Creates a new Rev_2Arm. */
  public Arm() {
    m_MPRatio = 15d;
    m_j1Ratio = 48d / 17d * m_MPRatio;
    m_j2Ratio = 36d / 17d * m_MPRatio;

    // ArmMotorFirstP.setPosition(High);
    m_rJoint1.setControl(new Follower(m_lJoint1.getDeviceID(), true));
    m_rJoint2.setControl(new Follower(m_lJoint2.getDeviceID(), true));

    m_lJoint1.setPosition(0);
    m_lJoint2.setPosition(0);

    TalonFXConfiguration armConfigs = new TalonFXConfiguration();
    armConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    m_lJoint1.getConfigurator().apply(armConfigs);
    m_lJoint2.getConfigurator().apply(armConfigs);

    var l_joint1Configs = new Slot0Configs();
    l_joint1Configs.kS = 0.15;
    l_joint1Configs.kV = 0.25;
    l_joint1Configs.kA = 0.01;
    l_joint1Configs.kP = 0.5;
    l_joint1Configs.kI = 0;
    l_joint1Configs.kD = 0;

    var l1_motionMagicConfigs = new TalonFXConfiguration().MotionMagic;
    l1_motionMagicConfigs.MotionMagicCruiseVelocity = 25; // Target cruise velocity of 80 rps
    l1_motionMagicConfigs.MotionMagicAcceleration = 60; // Target acceleration of 160 rps/s (0.5 seconds)
    l1_motionMagicConfigs.MotionMagicJerk = 100; // Target jerk of 1600 rps/s/s (0.1 seconds)

    // apply gains, 50 ms total timeout
    m_lJoint1.getConfigurator().apply(l_joint1Configs, 0.050);
    // m_lJoint1.getConfigurator().apply(l1_motionMagicConfigs, 0.050);

    var l_joint2Configs = new Slot0Configs();
    l_joint2Configs.kS = 0.15;
    l_joint2Configs.kV = 0.25;
    l_joint2Configs.kA = 0.05;
    l_joint2Configs.kP = 0.65;
    l_joint2Configs.kI = 0;
    l_joint2Configs.kD = 0;

    var l2_motionMagicConfigs = new TalonFXConfiguration().MotionMagic;
    l2_motionMagicConfigs.MotionMagicCruiseVelocity = 40; // Target cruise velocity of 80 rps
    l2_motionMagicConfigs.MotionMagicAcceleration = 85; // Target acceleration of 160 rps/s (0.5 seconds)
    l2_motionMagicConfigs.MotionMagicJerk = 100; // Target jerk of 1600 rps/s/s (0.1 seconds)

    // apply gains, 50 ms total timeout
    m_lJoint2.getConfigurator().apply(l_joint2Configs, 0.050);
    // m_lJoint2.getConfigurator().apply(l2_motionMagicConfigs, 0.050);
  }

  @Override
  public void periodic() {
    // System.out.println(m_lJoint1.getPosition().getValue());
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
              // System.out.println(m_lJoint1.getPosition().getValue() -
              // (-j1ParDeg*m_j1Ratio)/(360d));
              return Math.abs(m_lJoint1.getPosition().getValue() - (-j1ParDeg * m_j1Ratio) / (360d)) <= 20;
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

  public Command goToAmp() {
      return goToDegSeq(100, 0, -70)
  }
}
