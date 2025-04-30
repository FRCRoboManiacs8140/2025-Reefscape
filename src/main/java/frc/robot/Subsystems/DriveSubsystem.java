package frc.robot.Subsystems;


// Imports for the DriveSubsystem class
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;

public class DriveSubsystem extends SubsystemBase {
     // Define DriveSubsystem class

    // Define SparkMax configuration for normal and inverted motors
    SparkMaxConfig driveConfignormal = new SparkMaxConfig();
    SparkMaxConfig driveConfiginverted = new SparkMaxConfig();


     // Define motors
  private static final SparkMax leftFront = new SparkMax(6, MotorType.kBrushless);
  private static final SparkMax rightFront = new SparkMax(1, MotorType.kBrushless);
  private static final SparkMax leftBack = new SparkMax(4, MotorType.kBrushless);
  private static final SparkMax rightBack = new SparkMax(2, MotorType.kBrushless);
    // Define MecanumDrive
  private static final MecanumDrive drive = new MecanumDrive(leftFront, leftBack, rightFront, rightBack);

    // Allow for SparkMax motors to be accessed through Robot.java
    public static SparkMax leftFront() {
        return leftFront;
    }
    public static SparkMax getRightFront() {
        return rightFront;
    }
    public static SparkMax getLeftBack() {
        return leftBack;
    }
    public static SparkMax getRightBack() {
        return rightBack;
    }

    public static void drive(double xSpeed, double ySpeed, double rotation, Rotation2d gyroAngle) {
         // Define mecanum drive
    drive.driveCartesian(xSpeed, ySpeed, rotation, gyroAngle);

    }

    public static void brake() {
        // Set all motors to brake mode
        leftFront.set(0);
        rightFront.set(0);
        leftBack.set(0);
        rightBack.set(0);
    }
    
    public static void SmartDashboardDrive(){
     SmartDashboard.putNumber("Front Left", leftFront().get());
     SmartDashboard.putNumber("Front Right", rightFront.get());
     SmartDashboard.putNumber("Back Left", leftBack.get());
     SmartDashboard.putNumber("Back Right", rightBack.get());
     SmartDashboard.putNumber("Front Left Output Current", leftFront.getOutputCurrent());
     SmartDashboard.putNumber("Front Right Output Current", rightFront.getOutputCurrent());
     SmartDashboard.putNumber("Back Left Output Current", leftBack.getOutputCurrent());
     SmartDashboard.putNumber("Back Right Output Current", rightBack.getOutputCurrent());
     SmartDashboard.putNumber("Front Left Applied Output", leftFront.getAppliedOutput());
     SmartDashboard.putNumber("Front Right Applied Output", rightFront.getAppliedOutput());
     SmartDashboard.putNumber("Back Left Applied Output", leftBack.getAppliedOutput());
     SmartDashboard.putNumber("Back Right Applied Output", rightBack.getAppliedOutput());
     SmartDashboard.putNumber("Front Left Temperature", leftFront.getMotorTemperature());
     SmartDashboard.putNumber("Front Right Temperature", rightFront.getMotorTemperature());
     SmartDashboard.putNumber("Back Left Temperature", leftBack.getMotorTemperature());
     SmartDashboard.putNumber("Back Right Temperature", rightBack.getMotorTemperature());
    }
  
        
    // Set the idle mode for all motors to brake
         /** leftFront.configure<driveConfignormal, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters>
        leftBack.configure<driveConfignormal, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters>
        rightFront.configure<driveConfiginverted, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters>
        rightBack.configure<driveConfiginverted, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters>
*/
 


}
