package frc.robot.arm;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class ArmConfig {

  public static final Translation2d kOrigin =
      new Translation2d(Units.inchesToMeters(9.2), Units.inchesToMeters(20));

  public static final double kElbowGearing = 100.0;
  private static final double kElbowMassKg = 1.036005;
  private static final double kElbowLengthMeters = Units.inchesToMeters(19.0);
  private static final double kElbowMoiKgMetersSq = 0.08;
  private static final double kElbowCGRadiusMeters = kElbowLengthMeters / 2.0;
  private static final double kElbowMinAngleRad =
      -Math.PI; // Units.degreesToRadians(180.0 + 90 - 33.862);
  private static final double kElbowMaxAngleRad = Math.PI;
  private static final DCMotor kElbowGearbox =
      DCMotor.getFalcon500Foc(2).withReduction(kElbowGearing);

  public static final double kWristGearing = 100.0;
  private static final double kWristMassKg = 1.036005;
  private static final double kWristLengthMeters = Units.inchesToMeters(15.0);
  private static final double kWristMoiKgMetersSq = 0.08;
  private static final double kWristCGRadiusMeters = kWristLengthMeters / 2.0;
  private static final double kWristMinAngleRad = -Math.PI; // Units.degreesToRadians(5.682);
  private static final double kWristMaxAngleRad = Math.PI;
  private static final DCMotor kWristGearbox =
      DCMotor.getFalcon500Foc(2).withReduction(kWristGearing);

  public record JointConfig(
      double mass,
      double length,
      double moi,
      double cgRadius,
      double minAngle,
      double maxAngle,
      DCMotor gearbox) {}

  public static final JointConfig kElbowConfig =
      new JointConfig(
          kElbowMassKg,
          kElbowLengthMeters,
          kElbowMoiKgMetersSq,
          kElbowCGRadiusMeters,
          kElbowMinAngleRad,
          kElbowMaxAngleRad,
          kElbowGearbox);

  public static final JointConfig kWristConfig =
      new JointConfig(
          kWristMassKg,
          kWristLengthMeters,
          kWristMoiKgMetersSq,
          kWristCGRadiusMeters,
          kWristMinAngleRad,
          kWristMaxAngleRad,
          kWristGearbox);
}
