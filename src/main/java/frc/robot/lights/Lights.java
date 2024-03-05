package frc.robot.lights;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Function;
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
        .andThen(this.run(() -> {}))
        .ignoringDisable(true);
  }

  public Command setBlink() {
    final int[] blinkColor = colorToRGB(Color.kOrangeRed);
    final double kBlinkTimeSeconds = 1.0;
    return Commands.sequence(
            this.runOnce(
                () ->
                    m_ledController.animate(
                        new StrobeAnimation(
                            blinkColor[0], blinkColor[1], blinkColor[2], 0, 0.4, kLedCount))),
            Commands.waitSeconds(kBlinkTimeSeconds),
            this.runOnce(
                () ->
                    m_ledController.animate(
                        new StrobeAnimation(
                            blinkColor[0], blinkColor[1], blinkColor[2], 0, 0, kLedCount))),
            this.run(() -> {}))
        .ignoringDisable(true);
  }

  /**
   * @param color
   * @return an array of integers representing the RGB values of the given color: [0] is red, [1] is
   *     green, [2] is blue
   */
  private int[] colorToRGB(Color color) {
    final Function<Double, Integer> mapValue = value -> (int) (value * 255.0);
    return new int[] {
      mapValue.apply(color.red), mapValue.apply(color.green), mapValue.apply(color.blue)
    };
  }
}