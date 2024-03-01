package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.arm.Arm;
import frc.robot.arm.ArmSetpoints;
import frc.robot.climber.Climber;
import frc.robot.drivetrain.DriveToPose;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.feeder.Feeder;
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
  private final Arm m_arm;
  private final Climber m_climber;
  private final Shooter m_shooter;
  private final Feeder m_feeder;

  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_operatorController = new CommandXboxController(1);

  public Robot() {
    recordMetadeta();
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
        m_shooter = Subsystems.createTalonFXShooter();
        m_feeder = Subsystems.createTalonFXFeeder();
        break;
      case kSim:
        m_drivetrain = Subsystems.createTalonFXDrivetrain();
        m_vision = Subsystems.createFourCameraVision();
        m_intake = Subsystems.createBlankIntake();
        m_arm = Subsystems.createSimArm();
        m_climber = Subsystems.createBlankClimber();
        m_shooter = Subsystems.createSimShooter();
        m_feeder = Subsystems.createBlankFeeder();
        break;
      default:
        m_drivetrain = Subsystems.createBlankDrivetrain();
        m_vision = Subsystems.createBlankFourCameraVision();
        m_intake = Subsystems.createBlankIntake();
        m_arm = Subsystems.createBlankArm();
        m_climber = Subsystems.createBlankClimber();
        m_shooter = Subsystems.createBlankShooter();
        m_feeder = Subsystems.createBlankFeeder();
        break;
    }
    // NamedCommands.registerCommand("stop", m_shooter.Stop().withTimeout(0.5));
    // NamedCommands.registerCommand("sensorIntake", SensorIntake());
    NamedCommands.registerCommand("intake", m_intake.intake());
    NamedCommands.registerCommand("intakeOff", m_intake.idle());
    NamedCommands.registerCommand("enableShooter", new ScheduleCommand(m_shooter.runShooter()));
    NamedCommands.registerCommand("runShooter", m_shooter.runShooter().withTimeout(0.1));
    NamedCommands.registerCommand("ShooterOff", m_shooter.idle().withTimeout(1));
    NamedCommands.registerCommand("subShot", m_arm.goToSetpoint(-0.52, 2.083, 0.0, 0.0));
    // NamedCommands.registerCommand("feed", m_shooter.Feeder());
    // NamedCommands.registerCommand("ShooterDelay", m_shooter.ShooterDelay());
    NamedCommands.registerCommand("feederOn", m_feeder.feed().withTimeout(.3));
    NamedCommands.registerCommand("feederOnTest", m_feeder.feed());
    NamedCommands.registerCommand("feederOff", m_feeder.idle().withTimeout(1));
    // NamedCommands.registerCommand("ShooterOn", m_shooter.ShooterOn());
    NamedCommands.registerCommand("sensorIntake", Superstructure.sensorIntake(m_feeder, m_intake));
    NamedCommands.registerCommand("armDown", m_arm.goToSetpoint(ArmSetpoints.kStowed));
    NamedCommands.registerCommand("aimFromWingline", m_arm.goToSetpoint(-0.145, 2.784, 0.0, 0.0));

    m_autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());
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
    m_autoChooser.addOption(
        "Shooter SysId (Quasistatic Forward)",
        m_shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    m_autoChooser.addOption(
        "Shooter SysId (Quasistatic Reverse)",
        m_shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    m_autoChooser.addOption(
        "Shooter SysId (Dynamic Forward)", m_shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
    m_autoChooser.addOption(
        "Shooter SysId (Dynamic Reverse)", m_shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    m_autoChooser.addOption("sensor intake", Superstructure.sensorIntake(m_feeder, m_intake));
    m_drivetrain.setDefaultCommand(
        m_drivetrain.joystickDrive(
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getLeftX(),
            () -> -m_driverController.getRightX()));

    // m_feeder
    //     .hasNote
    //     .and(() -> DriverStation.isTeleopEnabled())
    //     .whileTrue(Commands.parallel(m_feeder.pullBack(), m_shooter.catchNote()))
    //     .onFalse(Commands.parallel(m_feeder.idle(), m_shooter.idle()).withTimeout(.3));

    // m_driverController.a().toggleOnTrue(m_arm.idleCoast());
    // m_driverController.y().toggleOnTrue(m_arm.goToSetpoint(-0.145, 2.784, 0.0, 0.0));

    m_driverController
        .rightTrigger()
        .whileTrue(Commands.parallel(m_intake.intake(), m_feeder.feed()))
        .onFalse(m_shooter.idle());

    // m_driverController
    //     .rightBumper()
    //     .whileTrue(Commands.sequence(m_intake.spit(), m_shooter.spit()));

    m_driverController.button(7).onTrue(m_drivetrain.zeroGyro());

    // m_driverController.a().whileTrue(m_climber.unwindWinch());
    // m_driverController.y().whileTrue(m_climber.windWinch());
    m_driverController.rightBumper().whileTrue(Superstructure.spit(m_shooter, m_feeder, m_intake));
    // First
    m_operatorController.leftStick().onTrue(m_arm.goToSetpoint(1.207, 3.274, 0, 0));

    m_driverController
        .b()
        .whileTrue(Commands.parallel(m_arm.idleCoast(), m_climber.windWinch()))
        .onFalse(Commands.parallel(m_shooter.idle(), m_arm.setBrake()));

    m_driverController.leftBumper().whileTrue(Superstructure.sensorIntake(m_feeder, m_intake));
    Command snapToAngle =
        new DriveToPose(
            new Pose2d(
                PoseEstimation.getInstance().getPose().getTranslation(),
                Rotation2d.fromDegrees(-32.98)),
            m_drivetrain);
    m_driverController.a().whileTrue(snapToAngle);

    // elbow = 1.147 rad
    // wrist = 3.805 rad
    m_operatorController
        .leftTrigger()
        .whileTrue(
            Commands.parallel(
                m_intake.feed(), m_shooter.runShooter(), m_arm.goToSetpoint(ArmSetpoints.kAmp)))
        .onFalse(m_arm.goToSetpoint(-0.547, 2.577, 0.0, 0.0));

    // m_operatorController.leftStick().onTrue(m_arm.goToSetpoint(ArmSetpoints.kClimbPreparation));

    m_operatorController.rightTrigger().onTrue(m_arm.goToSetpoint(ArmSetpoints.kStowed));

    m_operatorController
        .povUp()
        .onTrue(
            Commands.parallel(m_arm.goToSetpoint(-0.52, 2.083, 0.0, 0.0), m_shooter.runShooter()))
        .onFalse(m_arm.goToSetpoint(ArmSetpoints.kStowed));

    m_operatorController.y().whileTrue(m_shooter.runShooter());
    m_operatorController.a().whileTrue(m_shooter.ampShot());
    // wrist = 2.028
    m_operatorController
        .leftBumper()
        .onTrue(Superstructure.sensorCatch(m_shooter, m_feeder, m_intake, m_arm))
        .onFalse(m_arm.goToSetpoint(ArmSetpoints.kStowed));
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
    m_vision.updateSim(PoseEstimation.getInstance().getPose());
  }

  private void recordMetadeta() {
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }
  }
}
