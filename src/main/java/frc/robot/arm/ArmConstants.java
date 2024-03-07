package frc.robot.arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmConstants {
  private static final double kElbowGearboxReduction = 3.0 * 5.0 * 5.0;
  private static final double kElbowChainReduction = 36.0 / 22.0;
  public static final double kElbowReduction = kElbowGearboxReduction * kElbowChainReduction;
  public static final double kElbowLengthMeters = Units.inchesToMeters(19.0);
  public static final double kElbowMinAngleRad = -Math.PI;
  public static final double kElbowMaxAngleRad = Math.PI;
  public static final DCMotor kElbowGearbox = DCMotor.getFalcon500Foc(2);
  public static final double kElbowMassKg = Units.lbsToKilograms(7.0);
  public static final double kElbowMoiKgMetersSq =
      SingleJointedArmSim.estimateMOI(kElbowLengthMeters, kElbowMassKg);

  private static final double kWristGearboxReduction = 3.0 * 5.0;
  private static final double kWristChainReduction = 36.0 / 22.0;
  public static final double kWristReduction = kWristGearboxReduction * kWristChainReduction;
  public static final double kWristLengthMeters = Units.inchesToMeters(15.0);
  public static final double kWristMinAngleRad = 2.0 * -Math.PI;
  public static final double kWristMaxAngleRad = 2.0 * Math.PI;
  public static final DCMotor kWristGearbox = DCMotor.getFalcon500Foc(2);
  public static final double kWristMassKg = Units.lbsToKilograms(11.5);
  public static final double kWristMoiKgMetersSq =
      SingleJointedArmSim.estimateMOI(kWristLengthMeters, kWristMassKg);
}
