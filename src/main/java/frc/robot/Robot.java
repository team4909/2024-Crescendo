package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.PoseEstimation.AimingParameters;
import frc.robot.arm.Arm;
import frc.robot.arm.Arm.ArmSetpoints;
import frc.robot.climber.Climber;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.WheelRadiusCharacterization;
import frc.robot.drivetrain.WheelRadiusCharacterization.Direction;
import frc.robot.feeder.Feeder;
import frc.robot.intake.Intake;
import frc.robot.lights.Lights;
import frc.robot.shooter.Shooter;
import frc.robot.vision.GamePieceDetection;
import frc.robot.vision.Vision;
import java.util.HashMap;
import java.util.function.BiConsumer;
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
  private final Lights m_lights;
  private final GamePieceDetection m_gamePieceDetection;

  @SuppressWarnings("unused")
  private final PoseEstimation m_poseEstimation = PoseEstimation.getInstance();

  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_operatorController = new CommandXboxController(1);

  public Robot() {
    recordMetadeta();
    DriverStation.silenceJoystickConnectionWarning(true);
    SignalLogger.enableAutoLogging(false);
    switch (Constants.kCurrentMode) {
      case kReal:
        if (RobotBase.isSimulation()) {
          System.out.println("Wrong robot mode.");
          System.exit(1);
        }
        Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
        Logger.addDataReceiver(new NT4Publisher());
        PortForwarder.add(5800, "photonvision.local", 5800);
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
        m_vision = Subsystems.createFourCameraVision();
        m_intake = Subsystems.createSparkMAXIntake();
        m_arm = Subsystems.createTalonFXArm();
        m_climber = Subsystems.createSparkMAXClimber();
        m_shooter = Subsystems.createTalonFXShooter();
        m_feeder = Subsystems.createTalonFXFeeder();
        m_gamePieceDetection = Subsystems.createBlankGamePieceDetection();
        break;
      case kSim:
        m_drivetrain = Subsystems.createSimDrivetrain();
        m_vision = Subsystems.createFourCameraVision();
        m_intake = Subsystems.createSimIntake();
        m_arm = Subsystems.createSimArm();
        m_climber = Subsystems.createBlankClimber();
        m_shooter = Subsystems.createSimShooter();
        m_feeder = Subsystems.createSimFeeder();
        m_gamePieceDetection = Subsystems.createBlankGamePieceDetection();
        break;
      default:
        m_drivetrain = Subsystems.createBlankDrivetrain();
        m_vision = Subsystems.createBlankVision();
        m_intake = Subsystems.createBlankIntake();
        m_arm = Subsystems.createBlankArm();
        m_climber = Subsystems.createBlankClimber();
        m_shooter = Subsystems.createBlankShooter();
        m_feeder = Subsystems.createBlankFeeder();
        m_gamePieceDetection = Subsystems.createBlankGamePieceDetection();
        break;
    }
    m_lights = new Lights();
    NoteVisualizer.setWristPoseSupplier(m_arm.wristPoseSupplier);
    NoteVisualizer.resetNotes();
    NoteVisualizer.showStagedNotes();
    final Autos autos = new Autos(m_drivetrain, m_shooter, m_feeder, m_intake, m_arm, m_lights);
    NamedCommands.registerCommand("intake", m_intake.intake());
    NamedCommands.registerCommand("intakeOff", m_intake.idle());
    NamedCommands.registerCommand("enableShooter", new ScheduleCommand(m_shooter.runShooter()));
    NamedCommands.registerCommand("runShooter", m_shooter.runShooter().withTimeout(0.1));
    NamedCommands.registerCommand("subShot", m_arm.aimWrist(2.083));
    NamedCommands.registerCommand("feederOn", m_feeder.feed().withTimeout(.3));
    NamedCommands.registerCommand("feederOnTest", m_feeder.feed());
    NamedCommands.registerCommand("feederOff", m_feeder.idle().withTimeout(1));
    NamedCommands.registerCommand("sensorIntake", Superstructure.sensorIntake(m_feeder, m_intake));
    NamedCommands.registerCommand("armDown", m_arm.goToSetpoint(ArmSetpoints.kStowed));

    m_autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());
    m_autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        m_drivetrain.sysIdDriveQuasistatic(SysIdRoutine.Direction.kForward));
    m_autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        m_drivetrain.sysIdDriveQuasistatic(SysIdRoutine.Direction.kReverse));
    m_autoChooser.addOption(
        "Drive SysId (Dynamic Forward)",
        m_drivetrain.sysIdDriveDynamic(SysIdRoutine.Direction.kForward));
    m_autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)",
        m_drivetrain.sysIdDriveDynamic(SysIdRoutine.Direction.kReverse));
    m_autoChooser.addOption(
        "Rotation SysId (Quasistatic Forward)",
        m_drivetrain.sysIdRotationQuasistatic(SysIdRoutine.Direction.kForward));
    m_autoChooser.addOption(
        "Rotation SysId (Quasistatic Reverse)",
        m_drivetrain.sysIdRotationQuasistatic(SysIdRoutine.Direction.kReverse));
    m_autoChooser.addOption(
        "Rotation SysId (Dynamic Forward)",
        m_drivetrain.sysIdRotationDynamic(SysIdRoutine.Direction.kForward));
    m_autoChooser.addOption(
        "Rotation SysId (Dynamic Reverse)",
        m_drivetrain.sysIdRotationDynamic(SysIdRoutine.Direction.kReverse));
    m_autoChooser.addOption("Slip Current SysId", m_drivetrain.sysIdSlipCurrent());
    m_autoChooser.addOption(
        "Radius Characterization Clockwise",
        new WheelRadiusCharacterization(m_drivetrain, Direction.CLOCKWISE));
    m_autoChooser.addOption(
        "Radius Characterization Counterclockwise",
        new WheelRadiusCharacterization(m_drivetrain, Direction.COUNTER_CLOCKWISE));
    m_autoChooser.addOption("Shooter SysId", m_shooter.sysId());
    m_autoChooser.addOption(
        "Elbow SysId (Quasistatic Forward)",
        m_arm.sysIdElbowQuasistatic(SysIdRoutine.Direction.kForward));
    m_autoChooser.addOption(
        "Elbow SysId (Quasistatic Reverse)",
        m_arm.sysIdElbowQuasistatic(SysIdRoutine.Direction.kReverse));
    m_autoChooser.addOption(
        "Elbow SysId (Dynamic Forward)", m_arm.sysIdElbowDynamic(SysIdRoutine.Direction.kForward));
    m_autoChooser.addOption(
        "Elbow SysId (Dynamic Reverse)", m_arm.sysIdElbowDynamic(SysIdRoutine.Direction.kReverse));
    m_autoChooser.addOption(
        "Wrist SysId (Quasistatic Forward)",
        m_arm.sysIdWristQuasistatic(SysIdRoutine.Direction.kForward));
    m_autoChooser.addOption(
        "Wrist SysId (Quasistatic Reverse)",
        m_arm.sysIdWristQuasistatic(SysIdRoutine.Direction.kReverse));
    m_autoChooser.addOption(
        "Wrist SysId (Dynamic Forward)", m_arm.sysIdWristDynamic(SysIdRoutine.Direction.kForward));
    m_autoChooser.addOption(
        "Wrist SysId (Dynamic Reverse)", m_arm.sysIdWristDynamic(SysIdRoutine.Direction.kReverse));
    m_autoChooser.addOption("Start Signal Logger", Commands.runOnce(SignalLogger::start));
    m_autoChooser.addOption("End Signal Logger", Commands.runOnce(SignalLogger::stop));
    m_autoChooser.addOption("6 Piece", autos.sixPiece());
    m_drivetrain.setDefaultCommand(
        m_drivetrain.joystickDrive(
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getLeftX(),
            () -> -m_driverController.getRightX()));

    m_intake
        .hasIntookPieceSim
        .or(m_feeder.hasNote)
        .onTrue(Commands.runOnce(() -> NoteVisualizer.setHasNote(true)));
    m_feeder.hasNote.whileTrue(m_lights.setBlink(Color.kOrangeRed));
    m_drivetrain.inRangeOfGoal.whileTrue(m_lights.setBlink(Color.kBlue));

    m_driverController
        .rightTrigger()
        .whileTrue(Commands.parallel(m_intake.intake(), m_feeder.shoot()))
        .onFalse(Commands.runOnce(() -> m_shooter.getCurrentCommand().cancel()));
    m_driverController
        .leftTrigger()
        .whileTrue(Superstructure.aimAtGoal(m_drivetrain, m_shooter, m_arm, m_lights));

    m_driverController.start().onTrue(m_drivetrain.zeroGyro());
    m_operatorController
        .a()
        .whileTrue(m_arm.aimElbowForTuning(() -> -m_operatorController.getLeftY()))
        .onFalse(m_arm.holdSetpoint());
    m_operatorController
        .b()
        .whileTrue(m_arm.aimWristForTuning(() -> -m_operatorController.getLeftY()))
        .onFalse(m_arm.holdSetpoint());
    m_driverController.rightBumper().whileTrue(Superstructure.spit(m_shooter, m_feeder, m_intake));
    // m_operatorController.leftStick().onTrue(m_arm.goToSetpoint(ArmSetpoints.kClimb));
    m_driverController.a().onTrue(m_arm.goToSetpoint(ArmSetpoints.kClimb));
    m_driverController.b().onTrue(m_arm.goToSetpoint(ArmSetpoints.kTrap));
    m_driverController.b().whileTrue(Commands.parallel(m_climber.windWinch()));

    // m_driverController.x().onTrue(m_arm.goToSetpoint(ArmSetpoints.kTrap));
    m_driverController.y().whileTrue(Commands.parallel(m_climber.windWinch()));

    m_driverController.povRight().onTrue(m_shooter.trap());

    m_driverController.leftBumper().whileTrue(Superstructure.sensorIntake(m_feeder, m_intake));
    m_operatorController
        .leftTrigger()
        .whileTrue(
            Commands.parallel(
                m_intake.feed(), m_shooter.ampShot(), m_arm.goToSetpoint(ArmSetpoints.kAmp)))
        .onFalse(m_arm.goToSetpoint(ArmSetpoints.kStowed));
    m_operatorController.rightTrigger().onTrue(m_arm.goToSetpoint(ArmSetpoints.kStowed));
    m_operatorController
        .povUp()
        .onTrue(Commands.parallel(m_arm.aimWrist(2.083), m_shooter.runShooter()))
        .onFalse(m_arm.goToSetpoint(ArmSetpoints.kStowed));

    m_operatorController.y().whileTrue(m_shooter.runShooter());
    m_operatorController
        .leftBumper()
        .onTrue(Superstructure.sensorCatch(m_shooter, m_feeder, m_intake, m_arm))
        .onFalse(m_arm.goToSetpoint(ArmSetpoints.kStowed));

    final HashMap<String, Integer> commandCounts = new HashMap<>();
    final BiConsumer<Command, Boolean> logCommandConsumer =
        (Command command, Boolean active) -> {
          final String name = command.getName();
          final int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
          commandCounts.put(name, count);
          Logger.recordOutput(
              "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
          Logger.recordOutput("CommandsAll/" + name, count > 0);
        };
    CommandScheduler.getInstance()
        .onCommandInitialize(command -> logCommandConsumer.accept(command, true));
    CommandScheduler.getInstance()
        .onCommandFinish(command -> logCommandConsumer.accept(command, false));
    CommandScheduler.getInstance()
        .onCommandInterrupt(command -> logCommandConsumer.accept(command, false));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_vision.periodic();
    m_gamePieceDetection.periodic();
    NoteVisualizer.showHeldNotes();
    logAimingParameters();
  }

  @Override
  public void autonomousInit() {
    NoteVisualizer.resetNotes();
    NoteVisualizer.setHasNote(true);
    m_intake.resetSimIntookPieces();

    Command autonomousCommand = m_autoChooser.get();
    if (autonomousCommand != null) autonomousCommand.schedule();
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
    if (Constants.kIsSim) m_vision.updateSim(PoseEstimation.getInstance().getPose());
    NoteVisualizer.showStagedNotes();
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

  private void logAimingParameters() {
    final AimingParameters aimingParameters = PoseEstimation.getInstance().getAimingParameters();
    Logger.recordOutput(
        "PoseEstimation/AimingParameters/DriveHeading", aimingParameters.driveHeading());
    Logger.recordOutput("PoseEstimation/AimingParameters/ArmAngle", aimingParameters.armAngle());
    Logger.recordOutput(
        "PoseEstimation/AimingParameters/DriveFeedVelocity", aimingParameters.driveFeedVelocity());
    Logger.recordOutput(
        "PoseEstimation/AimingParameters/EffectiveDistance",
        PoseEstimation.getInstance().getAimingParameters().effectiveDistance());
    Logger.recordOutput(
        "PoseEstimation/AimingParameters/AimingJointIndex", aimingParameters.aimingJointIndex());
  }
}
