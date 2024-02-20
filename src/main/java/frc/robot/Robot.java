package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.arm.Arm;
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
  private final Arm m_arm = new Arm();

  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_operatorController = new CommandXboxController(1);
  private boolean speakerShot = false;

  public Robot() {
    recordMetadeta();

    switch (Constants.kCurrentMode) {
      case kReal:
        // TODO find out why this causes weird errors
        // Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;
      case kSim:
        DriverStation.silenceJoystickConnectionWarning(true);
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
        m_vision = Subsystems.createFourCameraVision();
        m_intake = Subsystems.createSparkMAXIntake();
        break;
      case kSim:
        m_drivetrain = Subsystems.createTalonFXDrivetrain();
        m_vision = Subsystems.createFourCameraVision();
        m_intake = Subsystems.createBlankIntake();
        break;
      default:
        m_drivetrain = Subsystems.createBlankDrivetrain();

        m_vision = Subsystems.createBlankFourCameraVision();
        m_intake = Subsystems.createBlankIntake();
        break;
    }
    m_vision.setVisionPoseConsumer(m_drivetrain.getVisionPoseConsumer());
    m_drivetrain.setDefaultCommand(
        m_drivetrain.joystickDrive(
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getLeftX(),
            // This needs to be getRawAxis(2) when using sim on a Mac
            () -> -m_driverController.getRightX()));
    // NamedCommands.registerCommand("stop", m_shooter.Stop().withTimeout(0.5));
    // NamedCommands.registerCommand("sensorIntake", SensorIntake());
    NamedCommands.registerCommand("intake", m_intake.intake());
    NamedCommands.registerCommand("shoot", m_shooter.Shoot().withTimeout(2));
    // NamedCommands.registerCommand("sub shot", m_arm.goToSubwoofer());
    NamedCommands.registerCommand("arm down", m_arm.goDown());
    NamedCommands.registerCommand("feed", m_shooter.Feeder());
    NamedCommands.registerCommand("ShooterDelay", m_shooter.ShooterDelay());
    NamedCommands.registerCommand("FeederOn", m_shooter.FeederOn());
    NamedCommands.registerCommand("ShooterOn", m_shooter.ShooterOn());
    NamedCommands.registerCommand("SensorIntake", SensorIntake());
    // NamedCommands.registerCommand("ArmDown", m_arm.goDown());
    NamedCommands.registerCommand("goDownAuto", m_arm.goToDegSeq(10, 0, -2));
    NamedCommands.registerCommand("2ndNoteShot", m_arm.goToDeg(10, 12));
    NamedCommands.registerCommand(
        "Stop Intake Shooter Feeder",
        Commands.sequence(m_intake.idle(), m_shooter.StopRepeatedly()));
    NamedCommands.registerCommand("Arm Go Down", m_arm.goDown());

    m_autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());

    m_intake.setDefaultCommand(m_intake.idle().repeatedly());
    m_shooter.setDefaultCommand(m_shooter.StopRepeatedly());
    // m_arm.setDefaultCommand(m_arm.goToDeg(20, 25));
    // m_arm.setDefaultCommand(m_arm.goDown());

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

    // ____________________driverController_______________________\\
    m_driverController
        .rightTrigger()
        .onTrue(new ParallelRaceGroup(m_intake.intake(), m_shooter.Feeder()))
        .onFalse(new InstantCommand(() -> speakerShot = false));

    m_driverController
        .rightBumper()
        .whileTrue(Commands.sequence(m_intake.spit(), m_shooter.FeederOut().repeatedly()))
        .onFalse(Commands.sequence(m_shooter.Stop(), m_intake.idle()));

    m_driverController.button(7).onTrue(m_drivetrain.zeroGyro());

    m_driverController
        .leftBumper()
        .onTrue(new ParallelCommandGroup(m_intake.intake(), m_shooter.FeederOn()).repeatedly())
        .onFalse(
            new ParallelRaceGroup(m_shooter.PullBack(), m_intake.intake().withTimeout(0.25))
                .finallyDo(() -> m_shooter.StopRepeatedly()));

    // ___________________OperatorController______________________\\
    m_operatorController
        .leftTrigger()
        .onTrue(Commands.parallel(m_intake.feed(), m_arm.goToAmp(), m_shooter.ShooterOn()))
        .onFalse(Commands.sequence(m_arm.goDown(), m_shooter.ShooterOff()));

    // prep for climb
    m_operatorController.rightBumper().onTrue(m_arm.goToDegSeq(110, 0, 0)).onFalse(m_arm.goDown());

    m_operatorController.rightTrigger().onTrue(m_arm.goDown()).onFalse(m_arm.goDown());

    m_operatorController
        .povUp()
        .onTrue(
            new ParallelCommandGroup(
                m_arm.goToDeg(20, 25),
                new InstantCommand(() -> speakerShot = true),
                m_shooter.ShooterOn()))
        .onFalse(Commands.sequence(m_arm.goDown(), m_shooter.ShooterOff()));

    // turn on the shooter wheels
    m_operatorController.y().onTrue(m_shooter.Shoot());

    m_operatorController
        .leftBumper()
        .onTrue(new ParallelCommandGroup(m_arm.goToDeg(0, 20), m_shooter.Catch().repeatedly()))
        .onFalse(new ParallelCommandGroup(m_arm.goDown(), m_shooter.StopRepeatedly()));
  }

  public Command SensorIntake() {
    return new ParallelRaceGroup(
            new RepeatCommand(m_intake.intake()),
            new RepeatCommand(m_shooter.FeederOn())
                .until(
                    () -> {
                      return m_shooter.hasNote();
                    }))
        .andThen(Commands.sequence(m_intake.idle(), m_shooter.FeederOff()));
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
