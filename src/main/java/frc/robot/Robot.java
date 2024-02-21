package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.arm.Arm;
import frc.robot.arm.ArmSetpoints;
import frc.robot.climber.Climber;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.intake.Intake;
import frc.robot.shooter.Shooter;
import frc.robot.vision.Vision;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {

  private final LoggedDashboardChooser<Command> m_autoChooser;
  private final Drivetrain m_drivetrain;
  private final Vision m_vision;
  private final Intake m_intake;
  private final Shooter m_shooter = new Shooter();
  private final Arm m_arm;
  private final Climber m_climber;

  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_operatorController = new CommandXboxController(1);

  public Robot() {
    // recordMetadeta();
    DriverStation.silenceJoystickConnectionWarning(true);
    switch (Constants.kCurrentMode) {
      case kReal:
        // TODO find out why this causes weird errors
        // Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;
      case kSim:
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;
      case kReplay:
        setUseTiming(false);
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    Logger.start();

    switch (Constants.kCurrentMode) {
      case kReal:
        m_drivetrain = Subsystems.createTalonFXDrivetrain();
        m_vision = Subsystems.createBlankFourCameraVision();
        m_intake = Subsystems.createSparkMAXIntake();
        m_arm = Subsystems.createTalonFXArm();
        m_climber = Subsystems.createSparkMAXClimber();
        break;
      case kSim:
        m_drivetrain = Subsystems.createTalonFXDrivetrain();
        m_vision = Subsystems.createFourCameraVision();
        m_intake = Subsystems.createBlankIntake();
        m_arm = Subsystems.createSimArm();
        m_climber = Subsystems.createBlankClimber();
        break;
      default:
        m_drivetrain = Subsystems.createBlankDrivetrain();
        m_vision = Subsystems.createBlankFourCameraVision();
        m_intake = Subsystems.createBlankIntake();
        m_arm = Subsystems.createBlankArm();
        m_climber = Subsystems.createBlankClimber();
        break;
    }
    m_vision.setVisionPoseConsumer(m_drivetrain.getVisionPoseConsumer());
    // NamedCommands.registerCommand("stop", m_shooter.Stop().withTimeout(0.5));
    // NamedCommands.registerCommand("sensorIntake", SensorIntake());
    NamedCommands.registerCommand("intake", m_intake.intake());
    NamedCommands.registerCommand("shoot", m_shooter.Shoot().withTimeout(2));
    // NamedCommands.registerCommand("sub shot", m_arm.goToSubwoofer());
    NamedCommands.registerCommand("feed", m_shooter.Feeder());
    NamedCommands.registerCommand("ShooterDelay", m_shooter.ShooterDelay());
    NamedCommands.registerCommand("FeederOn", m_shooter.FeederOn());
    NamedCommands.registerCommand("ShooterOn", m_shooter.ShooterOn());
    NamedCommands.registerCommand("SensorIntake", SensorIntake());

    m_autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());
    m_vision.setVisionPoseConsumer(m_drivetrain.getVisionPoseConsumer());
    m_autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        m_drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    m_autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        m_drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    m_autoChooser.addOption(
        "Drive SysId (Dynamic Forward)",
        m_drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
    m_autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)",
        m_drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    m_drivetrain.setDefaultCommand(
        m_drivetrain.joystickDrive(
            () -> m_driverController.getLeftY(),
            () -> m_driverController.getLeftX(),
            () -> m_driverController.getRightX()));

    m_driverController
        .rightTrigger()
        .onTrue(new ParallelRaceGroup(m_intake.intake(), m_shooter.Feeder()));

    m_driverController
        .rightBumper()
        .whileTrue(Commands.sequence(m_intake.spit(), m_shooter.FeederOut().repeatedly()))
        .onFalse(Commands.sequence(m_shooter.Stop(), m_intake.idle()));

    m_driverController.button(7).onTrue(m_drivetrain.zeroGyro());

    // Unwinch
    m_driverController.a().whileTrue(m_climber.release()).onFalse(m_climber.idle());

    // Winch
    m_driverController.y().whileTrue(m_climber.winchDown()).onFalse(m_climber.idle());

    // First
    m_driverController.povUp().onTrue(m_arm.goToSetpoint(1.207, 3.274, 0, 0));

    // Climb
    m_driverController
        .b()
        .onTrue(new ParallelCommandGroup(m_arm.climb(), m_climber.winchDown()))
        .onFalse(m_arm.setBrake());

    m_driverController.leftBumper().whileTrue(SensorIntake());

    m_driverController.povDown().whileTrue(m_arm.setCoast()).onFalse(m_arm.setBrake());

    // elbow = 1.147 rad
    // wrist = 3.805 rad
    m_operatorController
        .leftTrigger()
        .whileTrue(Commands.parallel(m_intake.feed(), m_arm.goToSetpoint(1.147, 3.805, 0.0, 0.0)))
        .onFalse(
            Commands.sequence(m_arm.goToSetpoint(ArmSetpoints.kStowed), m_shooter.ShooterOff()));

    m_operatorController.rightBumper().onTrue(m_arm.goToSetpoint(ArmSetpoints.kClimbPreparation));
    m_operatorController.leftStick().whileTrue(m_arm.climb());

    m_operatorController.rightTrigger().onTrue(m_arm.goToSetpoint(ArmSetpoints.kStowed));

    m_operatorController
        .povUp()
        .onTrue(
            new ParallelCommandGroup(
                m_arm.goToSetpoint(ArmSetpoints.kSubwoofer), m_shooter.ShooterOn()))
        .onFalse(
            Commands.sequence(m_arm.goToSetpoint(ArmSetpoints.kStowed), m_shooter.ShooterOff()));

    m_operatorController.y().onTrue(m_shooter.Shoot());
    // wrist = 2.028
    m_operatorController
        .leftBumper()
        .onTrue(new ParallelCommandGroup(m_arm.goToSetpoint(-0.558, 2.028, 0, 0), SensorCatch()))
        .onFalse(
            new ParallelCommandGroup(
                m_arm.goToSetpoint(ArmSetpoints.kStowed), m_shooter.StopRepeatedly()));
  }

  public Command SensorIntake() {
    return new ParallelDeadlineGroup(
        m_shooter.FeederOn().until(() -> m_shooter.hasNote()), m_intake.intake());
  }

  public Command SensorCatch() {
    return new ParallelDeadlineGroup(
        m_shooter.Catch().until(() -> m_shooter.hasNote()), m_intake.intake());
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and`
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_vision.periodic();

    SmartDashboard.putNumber("Intake/Current", m_shooter.getCurrent());
    SmartDashboard.putBoolean("Shooter/Sensor", m_shooter.hasNote());
  }

  @Override
  public void autonomousInit() {
    Command autonomousCommand = m_autoChooser.get();
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {
    m_vision.updateSim(m_drivetrain.getPose());
  }

  // private void recordMetadeta() {
  //   Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
  //   Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
  //   Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
  //   Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
  //   Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
  //   switch (BuildConstants.DIRTY) {
  //     case 0:
  //       Logger.recordMetadata("GitDirty", "All changes committed");
  //       break;
  //     case 1:
  //       Logger.recordMetadata("GitDirty", "Uncomitted changes");
  //       break;
  //     default:
  //       Logger.recordMetadata("GitDirty", "Unknown");
  //       break;
  //   }
  // }
}
