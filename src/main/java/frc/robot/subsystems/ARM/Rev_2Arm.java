// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ARM;

// import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class Rev_2Arm extends SubsystemBase {
    private double gearRatio1; 
    private double gearRatio2;
    private double gearR;

  private TalonFX l_Joint1 = new TalonFX(13, "CANivore2");
  private TalonFX l_Joint2 = new TalonFX(15, "CANivore2");
  private TalonFX r_Joint1 = new TalonFX(14, "CANivore2");
  private TalonFX r_Joint2 = new TalonFX(16, "CANivore2");

  final MotionMagicVoltage m_request = new MotionMagicVoltage(0).withSlot(0);


  /** Creates a new Rev_2Arm. */ 
  public Rev_2Arm() {
    gearR = 15d;
    gearRatio1 = 48d/17d*gearR;
    gearRatio2 = 36d/17d*gearR;

    // ArmMotorFirstP.setPosition(High);
    r_Joint1.setControl(new Follower(l_Joint1.getDeviceID(), true));
    r_Joint2.setControl(new Follower(l_Joint2.getDeviceID(), true));

    l_Joint1.setPosition(0);
    l_Joint2.setPosition(0);

    l_Joint1.getConfigurator().apply(new TalonFXConfiguration());
    l_Joint2.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration brakeConfiguration = new TalonFXConfiguration();
    brakeConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    l_Joint1.getConfigurator().apply(brakeConfiguration);
    l_Joint2.getConfigurator().apply(brakeConfiguration);
    // lowMotorFollower.configFactoryDefault();
    // upperMotorFollower.configFactoryDefault();
    // lowMotor.setSmartCurrentLimit(40);
    // upperMotor.setSmartCurrentLimit(40);

    var l_joint1Configs = new Slot0Configs();
    l_joint1Configs.kS = 0.15;
    l_joint1Configs.kV = 0.1;
    l_joint1Configs.kA = 0.01;
    l_joint1Configs.kP = 0.5;
    l_joint1Configs.kI = 0;
    l_joint1Configs.kD = 0;

    var l1_motionMagicConfigs = new TalonFXConfiguration().MotionMagic;
    l1_motionMagicConfigs.MotionMagicCruiseVelocity = 10; // Target cruise velocity of 80 rps
    l1_motionMagicConfigs.MotionMagicAcceleration = 10; // Target acceleration of 160 rps/s (0.5 seconds)
    l1_motionMagicConfigs.MotionMagicJerk = 10; // Target jerk of 1600 rps/s/s (0.1 seconds)

    // apply gains, 50 ms total timeout
    l_Joint1.getConfigurator().apply(l_joint1Configs, 0.050);
    l_Joint1.getConfigurator().apply(l1_motionMagicConfigs, 0.050);

    var l_joint2Configs = new Slot0Configs();
    l_joint2Configs.kS = 0.15;
    l_joint2Configs.kV = 0.1;
    l_joint2Configs.kA = 0.05;
    l_joint2Configs.kP = 0.65;
    l_joint2Configs.kI = 0;
    l_joint2Configs.kD = 0;

    var l2_motionMagicConfigs = new TalonFXConfiguration().MotionMagic;
    l2_motionMagicConfigs.MotionMagicCruiseVelocity = 20; // Target cruise velocity of 80 rps
    l2_motionMagicConfigs.MotionMagicAcceleration = 30; // Target acceleration of 160 rps/s (0.5 seconds)
    l2_motionMagicConfigs.MotionMagicJerk = 30; // Target jerk of 1600 rps/s/s (0.1 seconds)

    // apply gains, 50 ms total timeout
    l_Joint1.getConfigurator().apply(l_joint2Configs, 0.050);
    l_Joint1.getConfigurator().apply(l2_motionMagicConfigs, 0.050);

    // apply gains, 50 ms total timeout
    l_Joint2.getConfigurator().apply(l_joint2Configs, 0.050);

    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println(l_joint2.getTorqueCurrent());
  }

  // public Command idle() {
  // return new InstantCommand(() -> {
  // lowMotor.setControl(m_request.withPosition(LowMotorConstants.IDLE));
  // upperMotor.setControl(m_request.withPosition(UpperMotorConstants.IDLE));
  // }, this);
  // }

  // public Command low() {
  //   return new InstantCommand(() -> {
  //     System.out.println("LOW");
  //     l_joint1.setControl(m_request.withVelocity(Motor1Vel));
  //     l_joint2.setControl(m_request.withVelocity(1));
  //   }, this);
  // }

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

  // public Command move() {
  //   return new InstantCommand(()-> {
  //     l_joint1.setControl(m_request.withVelocity(-Motor1Vel));
  //     l_joint2.setControl(m_request.withVelocity(-Motor2Vel));
  //   }, this);

      // l_joint1.setControl(m_request.withVelocity(Motor1Vel));
      // l_joint2.setControl(m_request.withVelocity(Motor2Vel));
  // }

  public Command goToDeg(TalonFX joint, double GR, double degree){
    return new InstantCommand(()-> {
      // l_joint1.setControl(m_request.withPosition(-(degree*GearRatio1)/(360d)));
      joint.setControl(m_request.withPosition((degree*GR)/(360d)));
    }, this);
  }

  //   public Command goToDegVel(TalonFX joint, double GR, double degree, double vel){
  //   return new InstantCommand(()-> {
  //     // l_joint1.setControl(m_request.withPosition(-(degree*GearRatio1)/(360d)));
  //     joint.setControl(m_request.withPosition((degree*GR)/(360d)).withVelocity(vel));
  //   }, this);
  // }

  // public Command goToDegW1J(){
  //   return goToDeg(l_joint1, GearRatio1, 60);
  // }

  public Command goToDeg(double j1Degrees, double j2Degrees){
    return new SequentialCommandGroup(
      goToDeg(l_Joint1, gearRatio1, -j1Degrees),
      goToDeg(l_Joint2, gearRatio2, j2Degrees));
  }

  //   public Command goToDegVel(double j1Degrees, double j2Degrees, double vel){
  //   return new SequentialCommandGroup(
  //     goToDegVel(l_joint1, GearRatio1, -j1Degrees, vel),
  //     goToDegVel(l_joint2, GearRatio2, j2Degrees, vel));
  // }

  public Command goToDegSeq(double j1ParDeg, double j2ParDeg, double j2SeqDeg){
    return new SequentialCommandGroup(
      goToDeg(j1ParDeg, j2ParDeg),
      new WaitUntilCommand(() -> {
        System.out.println(l_Joint1.getPosition().getValue() - (-j1ParDeg*gearRatio1)/(360d));
        return Math.abs(l_Joint1.getPosition().getValue() - (-j1ParDeg*gearRatio1)/(360d)) <= 5;
      }),
      goToDeg(j1ParDeg, j2ParDeg)
      );
  }

  public Command goDown(){
      return new SequentialCommandGroup(
      goToDeg(l_Joint2, gearRatio2, 0),
      new WaitUntilCommand(() -> {
        return Math.abs(l_Joint2.getPosition().getValue() - (0*gearRatio2)/(360d)) <= 5;
      }),
      goToDeg(0, 0)
      );
  }

  // public Command stop(){
  //   return new InstantCommand(()-> {
  //     l_joint1.setControl(m_request.withVelocity(0));
  //     l_joint2.setControl(m_request.withVelocity(0));
  //   }, this);
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
  public double convertAngleToMotorPosition(double armAngle) {
    double motorRotations = armAngle * ((double) 1 / (double) 360) * ((double) 15 / (double) 1); // Double make 0 go bye
                                                                                                 // bye and make it
                                                                                                 // happy :)
    return motorRotations;
  }

}