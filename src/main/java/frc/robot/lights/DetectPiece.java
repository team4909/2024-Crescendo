package frc.robot.lights;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DetectPiece extends Command {

  Trigger trigger;
  boolean lastState = false;
  Lights lights;

  public DetectPiece(Trigger trigger, Lights lights) {
    this.trigger = trigger;
    this.lights = lights;
    addRequirements(lights);
  }

  @Override
  public void initialize() {
    lights.setBlinkMethod(Color.kRed);
  }

  @Override
  public void execute() {

    // if (trigger.getAsBoolean() == lastState) {
    //   return;
    // }
    // lastState = trigger.getAsBoolean();
    if (trigger.getAsBoolean()) {
      lights.setBlinkMethod(Color.kGreen);
    } else {
      lights.setBlinkMethod(Color.kRed);
    }
  }
}
