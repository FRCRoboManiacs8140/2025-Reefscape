package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightControls {

  public static void getLimelightControls(MecanumDrive drive, XboxController drive_controller, ADIS16470_IMU gyro) {
    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    double botpose[] = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[6]);
    int id = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getNumber(0).intValue();
    double tagAngle = getTagAngle(id);

    double tkI = SmartDashboard.getNumber("travel_to_integral_PID", 0.);
    double tkP = SmartDashboard.getNumber("travel_to_proportional_PID", 0.005);
    double tkD = SmartDashboard.getNumber("travel_to_derivative_PID", 0);
    double kI = SmartDashboard.getNumber("strafe_to_integral_PID", 0.4);
    double kP = SmartDashboard.getNumber("strafe_to_proportional_PID", 0.5);
    double kD = SmartDashboard.getNumber("strafe_to_derivative_PID", 0.4);

    PIDController travelToController = new PIDController(tkP, tkI, tkD);
    travelToController.setIntegratorRange(-5, 5);
    PIDController strafeController = new PIDController(kP, kI, kD);
    strafeController.setIntegratorRange(-5, 5);
    strafeController.setIZone(1);
    PIDController turnController = new PIDController(.01, .2, 0);
    PIDController anglePreserve = new PIDController(.01, .1, 0);

    SmartDashboard.putNumber("Target X", tx);
    SmartDashboard.putNumber("Target y", ty);
    SmartDashboard.putNumber("Target Area", ta);
    SmartDashboard.putNumber("Target Present", tv);

    try {
      SmartDashboard.putNumber("Rotation", botpose[5] * 0.025);
    } catch (Exception e) {
    }
    
    double turn = tx * -.005;
    double strafe = tx * -.025;
    double movePoint = 5;
    double travelTo = (4 - ta) * (-0.01 / ta);
    SmartDashboard.putNumber("strafe value", strafeController.calculate(tx, 0));
    SmartDashboard.putNumber("travel value", travelToController.calculate(ta, movePoint));
    SmartDashboard.putNumber("locate", turn);
    SmartDashboard.putNumber("travel to", travelTo);

    Rotation2d gyroangle = Rotation2d.fromDegrees(-gyro.getAngle());
    double movement_sensetivity = SmartDashboard.getNumber("drive_reduction", 1);
    double turn_sensetivity = 1;
    /*if (drive_controller.getAButton()){
      gyro.reset();
    }*/
    if (drive_controller.getLeftBumper()) {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
      if (tv == 1) {
        drive.driveCartesian(travelTo, strafe, turn);
      }
    } else if (drive_controller.getRightBumper()) {
      double current_angle = gyro.getAngle();
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
      if (tv == 1) {
        try {
          drive.driveCartesian(0, MathUtil.clamp(strafeController.calculate(tx, 0), -0.5, 0.8), botpose[5] * .0005);
        } catch (Exception e) {
          drive.driveCartesian(drive_controller.getLeftY() * movement_sensetivity, -drive_controller.getLeftX() * movement_sensetivity, -drive_controller.getRightX() * turn_sensetivity, gyroangle);
        }
      }
    } else if (drive_controller.getBButton()) {
      double current_angle = gyro.getAngle();
      drive.driveCartesian(-.05, -0.3, anglePreserve.calculate(gyro.getRate(), 0), Rotation2d.fromDegrees(0));
    } else if (drive_controller.getXButton()) {
      double current_angle = gyro.getAngle();
      drive.driveCartesian(-.05, 0.3, anglePreserve.calculate(gyro.getRate(), 0), Rotation2d.fromDegrees(0));
    } else if (drive_controller.getYButton()) {
      if (tv == 1) {
        drive.driveCartesian(travelToController.calculate(ta, movePoint) * -1, 0, 0);
      }
    } else if (drive_controller.getLeftStickButton()) {
      drive.driveCartesian(0, 0, turnController.calculate(gyro.getAngle() % 360, tagAngle));
    } else if (drive_controller.getRightTriggerAxis() >= 0.2) {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(3);
      if (tv == 1) {
        try {
          drive.driveCartesian(0, MathUtil.clamp(strafeController.calculate(tx, 0), -0.5, 0.8), botpose[5] * .0005);
        } catch (Exception e) {
          drive.driveCartesian(drive_controller.getLeftY() * movement_sensetivity, -drive_controller.getLeftX() * movement_sensetivity, -drive_controller.getRightX() * turn_sensetivity, gyroangle);
        }
      }
    } else if (drive_controller.getLeftTriggerAxis() >= 0.2) {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(2);
      if (tv == 1) {
        try {
          drive.driveCartesian(0, MathUtil.clamp(strafeController.calculate(tx, 0), -0.5, 0.8), botpose[5] * .0005);
        } catch (Exception e) {
          drive.driveCartesian(drive_controller.getLeftY() * movement_sensetivity, -drive_controller.getLeftX() * movement_sensetivity, -drive_controller.getRightX() * turn_sensetivity, gyroangle);
        }
      }
    } else {
      drive.driveCartesian(drive_controller.getLeftY() * movement_sensetivity, -drive_controller.getLeftX() * movement_sensetivity, -drive_controller.getRightX() * turn_sensetivity, gyroangle);
    }
  }

  private static double getTagAngle(int id) {
    switch (id) {
      case 7:
      case 18:
        return 0;
      case 6:
      case 19:
        return 60;
      case 11:
      case 20:
        return 120;
      case 10:
      case 21:
        return 180;
      case 9:
      case 22:
        return 240;
      case 8:
      case 17:
        return 300;
      default:
        return 0;
    }
  }
}
