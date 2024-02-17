package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.arm.Arm;
import frc.robot.arm.ArmIO;
import frc.robot.arm.ArmIOSim;
import frc.robot.arm.ArmIOTalonFX;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.ImuIO;
import frc.robot.drivetrain.ImuIOPigeon2;
import frc.robot.drivetrain.ModuleIO;
import frc.robot.drivetrain.ModuleIOTalonFX;
import frc.robot.vision.Vision;
import frc.robot.vision.VisionIO;
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

  public static Drivetrain createBlankDrivetrain() {
    return new Drivetrain(
        new ImuIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
  }

  public static Vision createFourCameraVision() {
    return new Vision(
        new VisionIOPhotonVision(
            "front-cam",
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(-10.330118),
                    Units.inchesToMeters(10.317953),
                    Units.inchesToMeters(9.041033)),
                new Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(-42.237756),
                    Units.degreesToRadians(150.0)))));
  }

  public static Vision createBlankFourCameraVision() {
    return new Vision(new VisionIO() {}, new VisionIO() {}, new VisionIO() {}, new VisionIO() {});
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
}
