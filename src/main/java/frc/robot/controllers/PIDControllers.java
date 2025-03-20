package frc.robot.controllers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PIDControllers {
  public final PIDController travelToController;
  public final PIDController strafeController;
  public final PIDController anglePreserve;
  public final PIDController elevatorPID;
  public final PIDController elevatorBottomPID;

  public PIDControllers() {
    // Initialize PID controllers
    double tkI = 0.01;
    double tkP = 0.06;
    double tkD = 0.05;
    SmartDashboard.getNumber("travel_to_integral_PID", tkI);
    SmartDashboard.getNumber("travel_to_proportional_PID", tkP);
    SmartDashboard.getNumber("travel_to_derivative_PID", tkD);
    travelToController = new PIDController(tkP, tkI, tkD);
    travelToController.setIntegratorRange(-5, 5);
    
    double kI = 1; 
    double kP = 0.02;
    double kD =  0.5;
    SmartDashboard.getNumber("strafe_to_integral_PID", kI);
    SmartDashboard.getNumber("strafe_to_proportional_PID", kP);
    SmartDashboard.getNumber("strafe_to_derivative_PID", kD);
    strafeController = new PIDController(kP, kI, kD);
    strafeController.setIntegratorRange(-5, 5);
    strafeController.setIZone(1);

    anglePreserve = new PIDController(0.01, 0.1, 0);
    elevatorPID = new PIDController(0.08, 0.1, 0);
    elevatorPID.setIntegratorRange(-5, 5);
    elevatorBottomPID = new PIDController(0.025, 0, 0);
  }
}
