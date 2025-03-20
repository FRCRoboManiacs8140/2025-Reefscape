package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.AnalogGyro;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Rotation2d;

public class DriveSubsystem {
  private final MecanumDrive drive;
  private final AnalogGyro gyro;

  // Define motors
  private final SparkMax leftFront = new SparkMax(6, MotorType.kBrushless);
  private final SparkMax rightFront = new SparkMax(1, MotorType.kBrushless);
  private final SparkMax leftBack = new SparkMax(4, MotorType.kBrushless);
  private final SparkMax rightBack = new SparkMax(2, MotorType.kBrushless);

  public DriveSubsystem() {
    // Initialize motors and drive system
    drive = new MecanumDrive(leftFront, leftBack, rightFront, rightBack);
    gyro = new AnalogGyro(0);
  }

  public void drive(XboxController controller, AnalogGyro gyro) {
    // Implement drive logic
    // ...existing code...
  }
}
