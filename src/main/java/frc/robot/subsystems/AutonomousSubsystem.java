package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutonomousSubsystem {
  private String autoSelected;
  private String levelSelected;
  private double autonomousStartTime;

  public void init(String autoSelected, String levelSelected) {
    this.autoSelected = autoSelected;
    this.levelSelected = levelSelected;
    autonomousStartTime = Timer.getFPGATimestamp();
  }

  public void periodic() {
    double autoTime = SmartDashboard.getNumber("Time elapsed", 0);
    // Implement autonomous logic
    // ...existing code...
  }
}
