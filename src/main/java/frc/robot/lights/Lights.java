package frc.robot.lights;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Lights extends SubsystemBase {

  private final int kLedCount = 8;
  private final CANdle m_ledController;

  public Lights() {
    m_ledController = new CANdle(1, "rio");
    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.RGB;
    config.disableWhenLOS = true;
    config.brightnessScalar = 1;
    m_ledController.configAllSettings(config);
    setDefaultCommand(setRainbow());
  }

  @Override
  public void periodic() {
    Logger.recordOutput("CANdle/5VRailVoltage", m_ledController.get5VRailVoltage());
    Logger.recordOutput("CANdle/Current", m_ledController.getCurrent());
    Logger.recordOutput("CANdle/Temperature", m_ledController.getTemperature());
  }

  public Command setRainbow() {
    return this.runOnce(() -> m_ledController.animate(new RainbowAnimation(1.0, 0.1, kLedCount)))
        .ignoringDisable(true);
  }

  public Command setBlink() {
    return Commands.sequence(
            this.runOnce(
                () -> m_ledController.animate(new StrobeAnimation(255, 165, 0, 0, 0.4, kLedCount))),
            Commands.waitSeconds(2.0))
        .ignoringDisable(true);
  }
}
