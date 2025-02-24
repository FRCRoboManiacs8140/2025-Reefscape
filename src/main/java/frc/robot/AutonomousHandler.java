package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutonomousHandler {

  public static void handleAutonomous(String autoSelected, MecanumDrive drive, double autonomousStartTime) {
    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    double autoTime = SmartDashboard.getNumber("Time elapsed", 0);

    double tkI = SmartDashboard.getNumber("travel_to_integral_PID", 0.);
    double tkP = SmartDashboard.getNumber("travel_to_proportional_PID", 0);
    double tkD = SmartDashboard.getNumber("travel_to_derivative_PID", 0);
    double kI = SmartDashboard.getNumber("strafe_to_integral_PID", 0);
    double kP = SmartDashboard.getNumber("strafe_to_proportional_PID", 0);
    double kD = SmartDashboard.getNumber("strafe_to_derivative_PID", 0);

    PIDController travelToController = new PIDController(tkP, tkI, tkD);
    travelToController.setIntegratorRange(-5, 5);
    PIDController strafeController = new PIDController(kP, kI, kD);
    strafeController.setIntegratorRange(-5, 5);
    strafeController.setIZone(1);

    double movepoint = 1;
    boolean moveForward = false;
    double rotateTo = tx * -0.5;
    if (tv == 0) {
      tx = 100;
    }

    switch (autoSelected) {
      case "Secondary Auto":
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
        if (tv == 1 && (tx > -2 && tx < 2)) {
          moveForward = true;
        }
        if ((autoTime > 0 && autoTime < 1000) && !moveForward) {
          if (tv == 0) {
            drive.driveCartesian(0, 0, 0.3);
          }
          if (tx < -5 || tx > 5)
            drive.driveCartesian(0, 0, rotateTo);
        }
        if (moveForward) {
          if ((tv == 1) && !(tx > -2 && tx < 2)) {
            drive.driveCartesian(0, MathUtil.clamp(strafeController.calculate(tx, 0), -0, 0.5), 0);
          }
          if ((tv == 1) && !(ta > 1)) {
            drive.driveCartesian(MathUtil.clamp(-travelToController.calculate(ta, movepoint), -0.5, 0.5), 0, 0);
          }
          if ((tv == 1) && (tx == 0) && (ta > 1)) {
            moveForward = false;
            autoTime = 1000;
          }
        }
        break;
      case "Default":
        if (autoTime > 0 && autoTime < 2) {
          drive.driveCartesian(-0.25, 0, 0);
        }
        break;
    }
  }
}
