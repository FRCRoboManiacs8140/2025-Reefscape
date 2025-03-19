package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class IntakeSubsystem {
  private final SparkMax endEffectorLeft;
  private final SparkMax endEffectorRight;

  public IntakeSubsystem() {
    // Initialize intake motors
    // ...existing code...
    endEffectorLeft = new SparkMax(7, MotorType.kBrushless);
    endEffectorRight = new SparkMax(8, MotorType.kBrushless);
  }

  public void control(XboxController controller) {
    // Implement intake control logic
    // ...existing code...
  }
}
