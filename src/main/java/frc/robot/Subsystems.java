package frc.robot;

import edu.wpi.first.math.geometry.Transform3d;
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
    return new Vision(new VisionIOPhotonVision("front-cam", new Transform3d()));
  }

  public static Vision createBlankFourCameraVision() {
    return new Vision(new VisionIO() {}, new VisionIO() {}, new VisionIO() {}, new VisionIO() {});
  }
}
