package frc.robot.Subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase{

    // Defines camera variables 
    public static void getVariables() {
        // Boolean that is 1 if a target is detected, 0 if not
    final double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);

    // X angle distance from center of camera frame to center of target
    final double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

    // Y angle distance from center of camera frame to center of target
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);

    // Area of the camera frame that the object takes up, can be used to estimate
    // how close the object is
    double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

    double botpose[] = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose")
        .getDoubleArray(new double[6]);
}
    public static double getX() {
        // X angle distance from center of camera frame to center of target
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    }
    public static double getY() {
        // Y angle distance from center of camera frame to center of target
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    }
    public static double getArea() {
        // Area of the camera frame that the object takes up, can be used to estimate
        // how close the object is
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    }
    public static double getTV() {
        // Boolean that is 1 if a target is detected, 0 if not
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    }
    
}
