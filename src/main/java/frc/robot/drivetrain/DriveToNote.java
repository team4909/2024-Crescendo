package frc.robot.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.intake.Intake;
import frc.robot.lights.Lights;
import frc.robot.vision.GamePieceDetection;

public class DriveToNote extends Command {
private Drivetrain m_drivetrain;
private Lights m_lights;
private GamePieceDetection m_gamePieceDetection;

private PIDController m_thetaController;

private double m_horizontalOffset;
private double m_verticalOffset;

public DriveToNote(Drivetrain drivetrain, Lights lights, GamePieceDetection gamePieceDetection, Intake intake) {
m_drivetrain = drivetrain;
m_lights = lights;
m_gamePieceDetection = gamePieceDetection;

m_thetaController = new PIDController(5.0, 0.0, 0.0);

m_horizontalOffset = m_gamePieceDetection.getHorizontalOffsetDeg();
m_verticalOffset = m_gamePieceDetection.getVerticalOffsetDeg();

m_thetaController.setTolerance(1);
}

@Override
public void initialize() {}

@Override
public void execute() {
m_horizontalOffset = m_gamePieceDetection.getHorizontalOffsetDeg();
m_verticalOffset = m_gamePieceDetection.getVerticalOffsetDeg();
m_drivetrain.runVelocity(
new ChassisSpeeds(0.0, 0.0, m_thetaController.calculate(m_horizontalOffset, 0)));
// m_lights.setBlink(Color.kMidnightBlue);
}

@Override
public void end(boolean interrupted) {}

@Override
public boolean isFinished() {
return false;
}
}