package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLightSubsystem {
  private double tv;
  private double tx;
  private double ty;
  private double ta;
  private double[] botpose;
  private double[] campose;
  private int id;

  public void updateLimelightData() {
    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    botpose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[6]);
    campose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("campose").getDoubleArray(new double[6]);
    id = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getNumber(0).intValue();
  }

  public double getTv() {
    return tv;
  }

  public double getTx() {
    return tx;
  }

  public double getTy() {
    return ty;
  }

  public double getTa() {
    return ta;
  }

  public double[] getBotpose() {
    return botpose;
  }

  public double[] getCampose() {
    return campose;
  }

  public int getId() {
    return id;
  }

  public double getTagAngle(int id) {
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
