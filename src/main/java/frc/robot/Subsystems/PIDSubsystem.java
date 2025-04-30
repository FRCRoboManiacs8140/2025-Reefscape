package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.controller.PIDController;

public class PIDSubsystem extends SubsystemBase {

    PIDController travelToController = new PIDController(tkP, tkI, tkD);
    travelToController.setIntegratorRange(-5, 5);

    PIDController strafeController = new PIDController(kP, kI, kD);
    strafeController.setIntegratorRange(-5, 5);
    strafeController.setIZone(1);

    PIDController turnController = new PIDController(.02, .1, 0);
    double tagAngle = getTagAngle(id);

    PIDController anglePreserve = new PIDController(.01, 0, 0);

    PIDController elevatorPID = new PIDController(0.08, 0.25, 0);
    elevatorPID.setIntegratorRange(-5, 5);
}
