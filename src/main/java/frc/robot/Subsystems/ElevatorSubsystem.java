package frc.robot.Subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

  private final static SparkMax elevatorRight = new SparkMax(3, MotorType.kBrushless);
  private final static SparkMax elevatorLeft = new SparkMax(5, MotorType.kBrushless);

  private final static SparkMax endEffectorLeft = new SparkMax(7, MotorType.kBrushless);
  private final static SparkMax endEffectorRight = new SparkMax(8, MotorType.kBrushless);

    // private final for encoder
  private final static RelativeEncoder elevator_encoder = elevatorRight.getEncoder();

   public static void set(double speed) {
    elevatorRight.set(speed);
    elevatorLeft.set(speed);
   }

    public static void setEndEffector(double speed) {
        endEffectorLeft.set(speed);
        endEffectorRight.set(-speed);    
    }

    public static void stop() {
        elevatorRight.set(0);
        elevatorLeft.set(0);
    }

    public static void stopEndEffector() {
        endEffectorLeft.set(0);
        endEffectorRight.set(0);
    }

    public double getEncoder() {
        return elevatorRight.getEncoder().getPosition();
    }

    public static void resetEncoder() {
        elevatorRight.getEncoder().setPosition(0);
    }

    public static void scoreL1() {
        endEffectorLeft.set(0.5);
        endEffectorRight.set(-0.2);
    }

    public static void score(){
        endEffectorLeft.set(0.5);
        endEffectorRight.set(-0.5);
    }

    
}
