package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.drivetrain.Drivetrain;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {

  private final LoggedDashboardChooser<String> m_chooser =
      new LoggedDashboardChooser<>("Auto Choices");
  private final Drivetrain m_drivetrain;
  // private final Intake m_intake = new Intake();
  // private final Shooter m_shooter = new Shooter();
  // private TimeOfFlight mytimeofflight = new TimeOfFlight(12);

  private final CommandXboxController m_driverController = new CommandXboxController(0);

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
        break;
      case kSim:
        m_drivetrain = Subsystems.createTalonFXDrivetrain();
        break;
      default:
        m_drivetrain = Subsystems.createBlankDrivetrain();
        break;
    }
    // m_drivetrain.setDefaultCommand(m_drivetrain.testDrive());
    m_driverController.a().whileTrue(m_drivetrain.AutoAlign());
    // m_drivetrain.setDefaultCommand(
    //     m_drivetrain.joystickDrive(
    //         () -> -m_driverController.getLeftY(),
    //         () -> -m_driverController.getLeftX(),
    //         // This needs to be getRawAxis(2) when using sim on a Mac
    //         () -> -m_driverController.getRightX()));
    m_driverController.y().onTrue(m_drivetrain.zeroRotation());
    // m_driverController.b().onTrue(m_drivetrain.AutoAlign());

    // double sensorValue = SmartDashboard.getNumber("Distance", mytimeofflight.getRange());

    // m_driverController.rightTrigger().whileTrue(m_shooter.ShooterDelay());
    // m_driverController.leftTrigger().whileTrue(new RepeatCommand(m_shooter.Intake()));
    // m_driverController.x().whileTrue(m_intake.intake());
    // m_driverController.y().whileTrue(m_intake.Spit());
    // m_driverController.b().whileTrue(m_intake.Stop());
    // final double defaultStopDistance = 0;
    // m_driverController.leftBumper().onTrue(m_shooter.Shoot());
    // m_driverController.rightBumper().onTrue(m_shooter.Feeder());
    // m_driverController.b().onTrue(m_shooter.Stop());
    // SmartDashboard.putNumber("StopDistance", defaultStopDistance);

    // m_driverController
    //     .a()
    //     .onTrue(
    //         new SequentialCommandGroup(
    //                 new InstantCommand(() -> m_intake.intake()),
    //                 new InstantCommand(() -> m_shooter.Intake()))
    //             .until(
    //                 () -> {
    //                   return sensorValue
    //                       <= SmartDashboard.getNumber("StopDistance", defaultStopDistance);
    //                 }));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    new PathPlannerAuto("New Auto").schedule();
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
  public void simulationPeriodic() {}

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
