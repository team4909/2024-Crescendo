package frc.robot.arm;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.util.Units;

public class ArmIOSim implements ArmIO {
  private Vector<N4> elbowWristSimStates = VecBuilder.fill(Math.PI / 2, Math.PI / 2, 0.0, 0.0);
  private ArmModel model = new ArmModel(ArmConfig.kElbowConfig, ArmConfig.kWristConfig);
  private PIDController elbowController = new PIDController(80.0, 0.0, 0.0);
  private PIDController wristController = new PIDController(80.0, 0.0, 0.0);

  private double elbowAppliedVolts = 0.0;
  private double wristAppliedVolts = 0.0;

  public ArmIOSim() {}

  public void updateInputs(ArmIOInputs inputs) {

    elbowWristSimStates =
        model.simulate(
            elbowWristSimStates, VecBuilder.fill(elbowAppliedVolts, wristAppliedVolts), 0.02);

    inputs.elbowPositionRad = elbowWristSimStates.get(0, 0);
    inputs.elbowVelocityRadPerSec = elbowWristSimStates.get(2, 0);
    inputs.elbowAppliedVolts = elbowAppliedVolts;
    inputs.elbowCurrentAmps =
        new double[] {
          ArmConfig.kElbowConfig
              .gearbox()
              .getCurrent(elbowWristSimStates.get(2, 0), elbowAppliedVolts)
        };

    inputs.wristPositionRad = elbowWristSimStates.get(1, 0);
    inputs.wristVelocityRadPerSec = elbowWristSimStates.get(3, 0);
    inputs.wristAppliedVolts = wristAppliedVolts;
    inputs.wristCurrentAmps =
        new double[] {
          ArmConfig.kWristConfig
              .gearbox()
              .getCurrent(elbowWristSimStates.get(3, 0), wristAppliedVolts)
        };
  }

  public void setElbowRotatations(double rotations, double feedForwardAmps) {

    elbowAppliedVolts =
        elbowController.calculate(
                elbowWristSimStates.get(0, 0), Units.rotationsToRadians(rotations))
            + feedForwardAmps;
  }

  public void setWristRotatations(double rotations, double feedForwardAmps) {

    wristAppliedVolts =
        wristController.calculate(
                elbowWristSimStates.get(1, 0), Units.rotationsToRadians(rotations))
            + feedForwardAmps;
  }
}
