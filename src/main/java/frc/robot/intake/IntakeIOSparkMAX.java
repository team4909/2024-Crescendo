// package frc.robot.intake;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;

// import edu.wpi.first.math.util.Units;

// public class IntakeIOSparkMAX implements IntakeIO {
// private final CANSparkMax topRoller, bottomRoller, centeringBagMotors;

// private final double kRollerReduction = 1.0;
// private final RelativeEncoder m_topRollerEncoder, m_bottomRollerEncoder;

// public IntakeIOSparkMAX() {
// topRoller = new CANSparkMax(5, CANSparkMax.MotorType.kBrushless);
// bottomRoller = new CANSparkMax(6, CANSparkMax.MotorType.kBrushless);
// centeringBagMotors = new CANSparkMax(8, CANSparkMax.MotorType.kBrushed);

// m_topRollerEncoder = topRoller.getEncoder();
// m_bottomRollerEncoder = bottomRoller.getEncoder();

// topRoller.restoreFactoryDefaults();
// bottomRoller.restoreFactoryDefaults();

// centeringBagMotors.setSmartCurrentLimit(15);

// topRoller.setSmartCurrentLimit(40);
// bottomRoller.setSmartCurrentLimit(40);
// }

// @Override
// public void updateInputs(IntakeIOInputs Inputs) {
// Inputs.topRollerVelocityRadPerSec =
// Units.rotationsToRadians(m_topRollerEncoder.getVelocity() /
// kRollerReduction);
// }
// }
