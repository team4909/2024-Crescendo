package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.arm.Arm;
import frc.robot.arm.ArmIO;
import frc.robot.arm.ArmIOSim;
import frc.robot.arm.ArmIOTalonFX;
import frc.robot.climber.Climber;
import frc.robot.climber.ClimberIO;
import frc.robot.climber.ClimberIOSparkMAX;
import frc.robot.climber.ClimberIOTalonFX;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.ImuIO;
import frc.robot.drivetrain.ImuIOPigeon2;
import frc.robot.drivetrain.ModuleIO;
import frc.robot.drivetrain.ModuleIOTalonFX;
import frc.robot.feeder.Feeder;
import frc.robot.feeder.FeederIO;
import frc.robot.feeder.FeederIOSim;
import frc.robot.feeder.FeederIOTalonFX;
import frc.robot.intake.Intake;
import frc.robot.intake.IntakeIO;
import frc.robot.intake.IntakeIOSim;
import frc.robot.intake.IntakeIOSparkMAX;
import frc.robot.intake.IntakeIOTalonFX;
import frc.robot.shooter.Shooter;
import frc.robot.shooter.ShooterIO;
import frc.robot.shooter.ShooterIOSim;
import frc.robot.shooter.ShooterIOTalonFX;
import frc.robot.vision.GamePieceDetection;
import frc.robot.vision.GamePieceDetectionIO;
import frc.robot.vision.GamePieceDetectionIOLimelight;
import frc.robot.vision.Vision;
import frc.robot.vision.VisionIOPhotonVision;

public class Subsystems {

  public static Drivetrain createTalonFXDrivetrain() {
    return new Drivetrain(
        new ImuIOPigeon2(),
        new ModuleIOTalonFX(0),
        new ModuleIOTalonFX(1),
        new ModuleIOTalonFX(2),
        new ModuleIOTalonFX(3));
  }

  public static Drivetrain createSimDrivetrain() {
    return new Drivetrain(
        new ImuIO() {},
        new ModuleIOTalonFX(0),
        new ModuleIOTalonFX(1),
        new ModuleIOTalonFX(2),
        new ModuleIOTalonFX(3));
  }

  public static Drivetrain createBlankDrivetrain() {
    return new Drivetrain(
        new ImuIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
  }

  public static Vision createFourCameraVision() {
    return new Vision(
        new VisionIOPhotonVision(
            "back-left-cam",
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(-9.124 + 2.5),
                    Units.inchesToMeters(10.646),
                    Units.inchesToMeters(8.25)),
                new Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(-28.125),
                    Units.degreesToRadians(150.0)))),
        new VisionIOPhotonVision(
            "back-right-cam",
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(-9.124 + 2.5),
                    Units.inchesToMeters(-10.646),
                    Units.inchesToMeters(8.25)),
                new Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(-28.125),
                    Units.degreesToRadians(-150.0)))));
  }

  public static Vision createBlankVision() {
    return new Vision();
  }

  public static Intake createTalonFXIntake() {
    return new Intake(new IntakeIOTalonFX());
  }

  public static Intake createSparkMAXIntake() {
    return new Intake(new IntakeIOSparkMAX());
  }

  public static Intake createSimIntake() {
    return new Intake(new IntakeIOSim());
  }

  public static Intake createBlankIntake() {
    return new Intake(new IntakeIO() {});
  }

  public static Arm createTalonFXArm() {
    return new Arm(new ArmIOTalonFX());
  }

  public static Arm createSimArm() {
    return new Arm(new ArmIOSim());
  }

  public static Arm createBlankArm() {
    return new Arm(new ArmIO() {});
  }

  public static Climber createSparkMAXClimber() {
    return new Climber(new ClimberIOSparkMAX());
  }

  public static Climber createTalonFXClimber() {
    return new Climber(new ClimberIOTalonFX());
  }

  public static Climber createBlankClimber() {
    return new Climber(new ClimberIO() {});
  }

  public static Shooter createTalonFXShooter() {
    return new Shooter(new ShooterIOTalonFX());
  }

  public static Shooter createSimShooter() {
    return new Shooter(new ShooterIOSim());
  }

  public static Shooter createBlankShooter() {
    return new Shooter(new ShooterIO() {});
  }

  public static Feeder createTalonFXFeeder() {
    return new Feeder(new FeederIOTalonFX());
  }

  public static Feeder createSimFeeder() {
    return new Feeder(new FeederIOSim());
  }

  public static Feeder createBlankFeeder() {
    return new Feeder(new FeederIO() {});
  }

  public static GamePieceDetection createLimelightGamePieceDetection() {
    return new GamePieceDetection(new GamePieceDetectionIOLimelight() {});
  }

  public static GamePieceDetection createBlankGamePieceDetection() {
    return new GamePieceDetection(new GamePieceDetectionIO() {});
  }
}
