package frc.robot.arm;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmConfig {

  public static final Translation2d kOrigin =
      new Translation2d(Units.inchesToMeters(-9.2), Units.inchesToMeters(20));

  public static final double kElbowGearboxReduction = 5.0;
  public static final double kElbowChainReduction = 48.0 / 17.0;
  public static final double kElbowReduction = kElbowGearboxReduction * kElbowChainReduction;
  private static final double kElbowMassKg = 1.036005;
  private static final double kElbowLengthMeters = Units.inchesToMeters(19.0);
  private static final double kElbowMoiKgMetersSq =
      SingleJointedArmSim.estimateMOI(kElbowLengthMeters, kElbowMassKg);
  private static final double kElbowCGRadiusMeters = kElbowLengthMeters / 2.0;
  private static final double kElbowMinAngleRad = -Math.PI;
  private static final double kElbowMaxAngleRad = Math.PI;
  private static final DCMotor kElbowGearbox =
      DCMotor.getFalcon500Foc(2).withReduction(kElbowReduction);

  public static final double kWristGearboxReduction = 5.0;
  public static final double kWristChainReduction = 36.0 / 17.0;
  public static final double kWristReduction = kWristGearboxReduction * kWristChainReduction;
  private static final double kWristMassKg = 1.036005;
  private static final double kWristLengthMeters = Units.inchesToMeters(15.0);
  private static final double kWristMoiKgMetersSq =
      SingleJointedArmSim.estimateMOI(kWristLengthMeters, kWristMassKg);
  private static final double kWristCGRadiusMeters = kWristLengthMeters / 2.0;
  private static final double kWristMinAngleRad = -Math.PI;
  private static final double kWristMaxAngleRad = Math.PI;
  private static final DCMotor kWristGearbox =
      DCMotor.getFalcon500Foc(2).withReduction(kWristReduction);

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

  public static enum ArmSetpoints {
    kStowed(-5, 9),
    kTrap(19.8, 30);

    private final Translation2d m_setpoint;

    private ArmSetpoints(Translation2d setpoint) {
      m_setpoint = setpoint;
    }

    private ArmSetpoints(double elbowPositionInches, double wristPositionInches) {
      this(
          new Translation2d(
              Units.inchesToMeters(elbowPositionInches),
              Units.inchesToMeters(wristPositionInches)));
    }

    public Translation2d get() {
      return m_setpoint;
    }
  }
}
