package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;

public class ElevatorSubsystem {
  private final SparkMax elevatorRight;
  private final SparkMax elevatorLeft;
  private final RelativeEncoder elevatorEncoder;

  public ElevatorSubsystem() {
    // Initialize elevator motors and encoder
    // ...existing code...
    elevatorRight = new SparkMax(3, MotorType.kBrushless);
    elevatorLeft = new SparkMax(5, MotorType.kBrushless);
    elevatorEncoder = elevatorRight.getEncoder();
  }

  public void resetEncoder() {
    elevatorEncoder.setPosition(0);
  }

  public void control(XboxController controller) {
    // Implement elevator control logic
    // ...existing code...
  }
}
