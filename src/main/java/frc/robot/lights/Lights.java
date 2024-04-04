package frc.robot.lights;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj.DriverStation;
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

  private final int kLedCount = Constants.kIsViper ? 8 : 32;
  private final int kCandleLedOffset = Constants.kIsViper ? 0 : 8;
  private final CANdle m_ledController;

  public Lights() {
    m_ledController = new CANdle(1, "rio");
    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.RGB;
    config.disableWhenLOS = true;
    config.brightnessScalar = 1;
    m_ledController.configAllSettings(config);
    setDefaultCommand(idle());
  }

  @Override
  public void periodic() {
    if (Constants.kIsSim) return;
    Logger.recordOutput("CANdle/5VRailVoltage", m_ledController.get5VRailVoltage());
    Logger.recordOutput("CANdle/Current", m_ledController.getCurrent());
    Logger.recordOutput("CANdle/Temperature", m_ledController.getTemperature());
  }

  public Command idle() {
    return Commands.either(
            startGreenLarson().until(DriverStation::isEnabled),
            this.runOnce(() -> m_ledController.animate(new SingleFadeAnimation(0, 0, 0, 0, 0, 0)))
                .andThen(this.run(() -> {})),
            DriverStation::isDisabled)
        .ignoringDisable(true);
  }

  public Command startRainbow() {
    return this.runOnce(() -> m_ledController.animate(new RainbowAnimation(1.0, 0.1, kLedCount)))
        .andThen(this.run(() -> {}))
        .ignoringDisable(true);
  }

  public Command startGreenLarson() {
    final ColorRGB color = colorToRGB(Color.kGreen);
    return this.runOnce(
            () ->
                m_ledController.animate(
                    new LarsonAnimation(
                        color.red,
                        color.green,
                        color.blue,
                        0,
                        0.0,
                        kLedCount - kCandleLedOffset,
                        BounceMode.Center,
                        7,
                        kCandleLedOffset)))
        .andThen(this.run(() -> {}))
        .ignoringDisable(true);
  }

  public Command startBlink(Color color) {
    return Commands.sequence(this.runOnce(() -> setBlink(color)), this.run(() -> {}))
        .ignoringDisable(true);
  }

  public Command noteBlink() {
    return Commands.sequence(
            this.runOnce(() -> setBlink(Color.kOrangeRed)), this.run(() -> {}).withTimeout(2.0))
        .ignoringDisable(true);
  }

  public void setBlink(Color color) {
    final ColorRGB blinkColor = colorToRGB(color);
    m_ledController.animate(
        new StrobeAnimation(blinkColor.red, blinkColor.green, blinkColor.blue, 0, 0.4, kLedCount));
  }

  private ColorRGB colorToRGB(Color color) {
    final Function<Double, Integer> mapValue = value -> (int) (value * 255.0);
    return new ColorRGB(
        mapValue.apply(color.red), mapValue.apply(color.green), mapValue.apply(color.blue));
  }

  public Command showReadyToShootStatus(Trigger readyToShoot) {
    var state =
        new Object() {
          boolean lastTriggerState = false;
        };
    return new FunctionalCommand(
        () -> setBlink(Color.kRed),
        () -> {
          final boolean currentTriggerState = readyToShoot.getAsBoolean();
          if (state.lastTriggerState == currentTriggerState) return;
          if (readyToShoot.getAsBoolean()) setBlink(Color.kGreen);
          else setBlink(Color.kRed);
          state.lastTriggerState = readyToShoot.getAsBoolean();
        },
        (interrupted) -> startRainbow().schedule(),
        () -> false,
        this);
  }

  private record ColorRGB(int red, int green, int blue) {}
}
