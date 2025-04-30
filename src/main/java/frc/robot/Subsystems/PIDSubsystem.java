package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.CameraSubsystem;

public class PIDSubsystem extends SubsystemBase {

    // PID coefficients as class-level fields
    
    // values for travel to; PID
    private double tkI = 0;
    private double tkP = 0.005;
    private double tkD = 0;
    // values for strafe; PID
    private double kI = 0;
    private double kP = 0.2;
    private double kD = 0;

    // PID Controllers
    private PIDController travelToController;
    private PIDController strafeController;
    private PIDController anglePreserve;
    private PIDController elevatorPID;

    public PIDSubsystem(){
        // Initialize travelToController
        travelToController = new PIDController(tkP, tkI, tkD);
        travelToController.setIntegratorRange(-5, 5);

        // Initialize strafeController
        strafeController = new PIDController(kP, kI, kD);
        strafeController.setIntegratorRange(-5, 5);
        strafeController.setIZone(1);

        // Initialize anglePreserve
        anglePreserve = new PIDController(0.01, 0.0, 0.0);

        // Initialize elevatorPID
        elevatorPID = new PIDController(0.08, 0.25, 0.0);
        elevatorPID.setIntegratorRange(-5, 5);
    }

    // Method to update PID coefficients from SmartDashboard
    public void updatePIDCoefficients() {
        tkP = SmartDashboard.getNumber("travel_to_proportional_PID", tkP);
        tkI = SmartDashboard.getNumber("travel_to_integral_PID", tkI);
        tkD = SmartDashboard.getNumber("travel_to_derivative_PID", tkD);

        kP = SmartDashboard.getNumber("strafe_to_proportional_PID", kP);
        kI = SmartDashboard.getNumber("strafe_to_integral_PID", kI);
        kD = SmartDashboard.getNumber("strafe_to_derivative_PID", kD);

        // Update PID controllers with new coefficients
        travelToController.setP(tkP);
        travelToController.setI(tkI);
        travelToController.setD(tkD);

        strafeController.setP(kP);
        strafeController.setI(kI);
        strafeController.setD(kD);
    }

     // Getter methods for PID controllers
     public PIDController getTravelToController() {
        return travelToController;
    }

    public PIDController getStrafeController() {
        return strafeController;
    }

    public PIDController getAnglePreserve() {
        return anglePreserve;
    }

    public PIDController getElevatorPID() {
        return elevatorPID;
    }
}
