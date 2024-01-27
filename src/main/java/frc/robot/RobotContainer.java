package frc.robot;

import java.time.Instant;

import javax.swing.GroupLayout.ParallelGroup;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    public static TimeOfFlight mytimeofflight = new TimeOfFlight(12);

    /* Controllers */
    private final CommandXboxController driver = new CommandXboxController(0);
    // private CommandXboxController m_oppController = new CommandXboxController(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final Trigger zeroGyro = driver.start();
    private final Trigger robotCentric = driver.povUp();
    private final Trigger Intake = driver.x();
    private final Trigger Spit = driver.y();
    private final Trigger Stop = driver.b();
    private final Trigger Feeder = driver.rightBumper();
    private final Trigger Shoot = driver.leftBumper();
    // Opperator Buttons
    private final Trigger ShootButton = driver.rightTrigger();
    private final Trigger IntakeButton = driver.leftTrigger();

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Rev_1Shooter s_Shooter = new Rev_1Shooter();
    private final Rev_1Intake s_Intake = new Rev_1Intake();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        // Commands that can be used from path planner
        NamedCommands.registerCommand("ShooterDelay", s_Shooter.ShooterDelay().withTimeout(1));
        NamedCommands.registerCommand("Stop", s_Shooter.Stop());

        NamedCommands.registerCommand("SensorIntake", SensorIntake());
        // var jt = new JoystickTrigger(driver,
        // XboxController.Axis.kRightTrigger.value);
        // CommandXboxController m_oppController = new CommandXboxController(1);
        // SmartDashboard.putNumber("Distance", mytimeofflight.getRange());
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> -driver.getRawAxis(rotationAxis),
                        () -> robotCentric.getAsBoolean()));

        s_Shooter.setDefaultCommand(s_Shooter.Stop());
        s_Intake.setDefaultCommand(s_Intake.Stop());

        // Configure the button bindings
        configureButtonBindings();

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        ShootButton.whileTrue(s_Shooter.ShooterDelay());
        IntakeButton.whileTrue(new RepeatCommand(s_Shooter.Intake()));
        Intake.whileTrue(s_Intake.Intake());
        Spit.whileTrue(s_Intake.Spit());
        Stop.whileTrue(s_Intake.Stop());

        Shoot.onTrue(s_Shooter.Shoot());
        Feeder.onTrue(s_Shooter.Feeder());
        Stop.onTrue(s_Shooter.Stop());

        // System.out.println(mytimeofflight.getRange());
        driver.a().onTrue(SensorIntake());

    };

    public Command SensorIntake() {
        final double defaultStopDistance = 35;
        SmartDashboard.putNumber("StopDistance", defaultStopDistance); // this is here to make the value be editable on
                                                                       // the dashboard
        return new ParallelRaceGroup(
                new RepeatCommand(s_Intake.Intake()),
                new RepeatCommand(s_Shooter.Intake()).until(() -> {
                    return mytimeofflight.getRange() <= SmartDashboard.getNumber("StopDistance",
                            defaultStopDistance);
                }));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        // return new exampleAuto(s_Swerve);
        // return new PathPlannerAuto("Auto1");
        return new PathPlannerAuto("Copy of Auto3");
    }
}
