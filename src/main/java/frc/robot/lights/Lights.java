package frc.robot.lights;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
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
    if (Constants.kIsSim) return;
    Logger.recordOutput("CANdle/5VRailVoltage", m_ledController.get5VRailVoltage());
    Logger.recordOutput("CANdle/Current", m_ledController.getCurrent());
    Logger.recordOutput("CANdle/Temperature", m_ledController.getTemperature());
  }

  public Command setRainbow() {
    return this.runOnce(() -> m_ledController.animate(new RainbowAnimation(1.0, 0.1, kLedCount)))
        .andThen(this.run(() -> {}))
        .ignoringDisable(true);
  }

  public Command setBlink(Color color) {
    final int[] blinkColor = colorToRGB(color);
    return Commands.sequence(
            this.runOnce(
                () ->
                    m_ledController.animate(
                        new StrobeAnimation(
                            blinkColor[0], blinkColor[1], blinkColor[2], 0, 0.4, kLedCount))),
            this.run(() -> {}))
        .ignoringDisable(true);
  }

  public void setBlinkMethod(Color color) {
    final int[] blinkColor = colorToRGB(color);

    m_ledController.animate(
        new StrobeAnimation(blinkColor[0], blinkColor[1], blinkColor[2], 0, 0.4, kLedCount));
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

  public Command showReadyToShootStatus(Trigger readyToShoot) {
    var lastState =
        new Object() {
          boolean value = false;
        };
    return new FunctionalCommand(
        () -> setBlinkMethod(Color.kRed),
        () -> {
          boolean currentTriggerState = readyToShoot.getAsBoolean();
          if (lastState.value == currentTriggerState) return;
          if (readyToShoot.getAsBoolean()) setBlinkMethod(Color.kGreen);
          else setBlinkMethod(Color.kRed);
          lastState.value = readyToShoot.getAsBoolean();
        },
        (interrupted) -> {
          setRainbow();
        },
        () -> false,
        this);
  }
}
