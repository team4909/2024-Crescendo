// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ARM;

import java.lang.module.ModuleDescriptor.Requires;

// import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.Idle;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.CANVenom.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
    private double AngularVelocity;
    private double GearRatio1; 
    private double GearRatio2;
    private double Motor1Vel;
    private double Motor2Vel;
    private double GearR;

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

  private TalonFX l_joint1 = new TalonFX(13, "CANivore2");
  private TalonFX l_joint2 = new TalonFX(15, "CANivore2");
  private TalonFX r_joint1 = new TalonFX(14, "CANivore2");
  private TalonFX r_joint2 = new TalonFX(16, "CANivore2");

  final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);


  private final DutyCycleOut m_lMotor = new DutyCycleOut(1.0);
  private final DutyCycleOut m_rMotor = new DutyCycleOut(1.0);

  /** Creates a new Rev_2Arm. */
  public Rev_2Arm() {
    AngularVelocity = 1;
    GearR = 15d;
    GearRatio1 = 48d/17d*GearR;
    GearRatio2 = 36d/17d*GearR;

    // ArmMotorFirstP.setPosition(High);
    r_joint1.setControl(new Follower(l_joint1.getDeviceID(), true));
    r_joint2.setControl(new Follower(l_joint1.getDeviceID(), true));

    l_joint1.setPosition(0);
    l_joint2.setPosition(0);

    l_joint1.getConfigurator().apply(new TalonFXConfiguration());
    l_joint2.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration brakeConfiguration = new TalonFXConfiguration();
    brakeConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    l_joint1.getConfigurator().apply(brakeConfiguration);
    l_joint2.getConfigurator().apply(brakeConfiguration);
    // lowMotorFollower.configFactoryDefault();
    // upperMotorFollower.configFactoryDefault();
    // lowMotor.setSmartCurrentLimit(40);
    // upperMotor.setSmartCurrentLimit(40);

    var l_joint1Configs = new Slot0Configs();
    l_joint1Configs.kV = 0;
    l_joint1Configs.kP = 0.5;
    l_joint1Configs.kI = 0;
    l_joint1Configs.kD = 0;

    // apply gains, 50 ms total timeout
    l_joint1.getConfigurator().apply(l_joint1Configs, 0.050);

    var l_joint2Configs = new Slot0Configs();
    l_joint2Configs.kV = 0;
    l_joint2Configs.kP = 0.5;
    l_joint2Configs.kI = 0;
    l_joint2Configs.kD = 0;

    // apply gains, 50 ms total timeout
    l_joint2.getConfigurator().apply(l_joint2Configs, 0.050);

    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.println(l_joint1.getTorqueCurrent());
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
      l_joint1.setControl(m_request.withVelocity(Motor1Vel));
      l_joint2.setControl(m_request.withVelocity(1));
    }, this);
  }

  // public Command stop() {
  //   return new InstantCommand(() -> {
  //     l_joint1.setControl(new DutyCycleOut(0));
  //     l_joint2.setControl(new DutyCycleOut(0));
  //   }, this);
  // }

  // public Command podium() {
  //   return goToPosition(100, 200);
  // }

  // public Command low2() {
  //   return goToPosition(5, 20);
  // }

  // public Command goToPosition(double upperAngle, double lowerAngle) {
  //   // using defer to wait for the command to be scheduled before binding in the
  //   // angles
  //   return this.defer(() -> new InstantCommand(() -> {
  //     System.out.println("going to" + upperAngle + " lower:" + lowerAngle);
  //     System.out.println(convertAngleToMotorPosition(lowerAngle));
  //     l_joint1.setControl(m_request.withPosition(convertAngleToMotorPosition(lowerAngle)));
  //     l_joint2.setControl(m_request.withPosition(convertAngleToMotorPosition(upperAngle)));
  //   }, this));
  // }

  public Command move() {
    return new InstantCommand(()-> {
      l_joint1.setControl(m_request.withVelocity(-Motor1Vel));
      l_joint2.setControl(m_request.withVelocity(-Motor2Vel));
    }, this);

      // l_joint1.setControl(m_request.withVelocity(Motor1Vel));
      // l_joint2.setControl(m_request.withVelocity(Motor2Vel));
  }

  public Command goToDeg(double degree){
    return new InstantCommand(()-> {
      l_joint1.setControl(m_request.withPosition(-(degree*GearRatio1)/(360d)));
      l_joint2.setControl(m_request.withPosition(-(degree*GearRatio2)/(360d)));
    }, this);
  }

  public Command stop(){
    return new InstantCommand(()-> {
      l_joint1.setControl(m_request.withVelocity(0));
      l_joint2.setControl(m_request.withVelocity(0));
    }, this);
  }

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
  public double convertAngleToMotorPosition(double armAngle) {
    double motorRotations = armAngle * ((double) 1 / (double) 360) * ((double) 15 / (double) 1); // Double make 0 go bye
                                                                                                 // bye and make it
                                                                                                 // happy :)
    return motorRotations;
  }

}