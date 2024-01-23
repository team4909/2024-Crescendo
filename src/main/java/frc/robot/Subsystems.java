package frc.robot;

import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.ImuIO;
import frc.robot.drivetrain.ImuIOPigeon2;
import frc.robot.drivetrain.ModuleIO;
import frc.robot.drivetrain.ModuleIOTalonFX;

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
}
