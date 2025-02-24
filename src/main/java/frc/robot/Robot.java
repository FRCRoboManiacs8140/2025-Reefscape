// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root di rectory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kSecondaryAuto = "Secondary Auto";

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private final WPI_VictorSPX leftFront = new WPI_VictorSPX(3);
  private final WPI_VictorSPX rightFront = new WPI_VictorSPX(2);
  private final WPI_VictorSPX leftBack = new WPI_VictorSPX(1);
  private final WPI_VictorSPX rightBack = new WPI_VictorSPX(4);

  private final MecanumDrive drive = new MecanumDrive(leftFront, leftBack, rightFront, rightBack);
  private final XboxController drive_controller = new XboxController(0);
  private final ADIS16470_IMU gyro = new ADIS16470_IMU();

  public double autonomousStartTime, timeElapsed;
  public double strafeStartTime = 0;
  public double second = 1;
  public double lastSecond = 0;

  double current_angle;

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("Secondary Auto", kSecondaryAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    leftFront.setInverted(true);
    rightFront.setInverted(false);
    leftBack.setInverted(true);
    rightBack.setInverted(false);

    SmartDashboard.putNumber("drive_reduction", 1);
    CameraServer.startAutomaticCapture();

    SmartDashboard.putNumber("travel_to_integral_PID", 0.01);
    SmartDashboard.putNumber("travel_to_proportional_PID", 0.06);
    SmartDashboard.putNumber("travel_to_derivative_PID", 0.05);

    SmartDashboard.putNumber("strafe_to_integral_PID", 1);
    SmartDashboard.putNumber("strafe_to_proportional_PID", 0.02);
    SmartDashboard.putNumber("strafe_to_derivative_PID", 0.5);
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("current angle", current_angle);
    double gameTime = Timer.getFPGATimestamp() - autonomousStartTime;
    double strafeTime = Timer.getFPGATimestamp() - strafeStartTime;
    SmartDashboard.putNumber("Strafe Time", strafeTime);
    SmartDashboard.putNumber("Time Remaining", (150 - gameTime));
    SmartDashboard.putNumber("Time elapsed", gameTime);
  }

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
    SmartDashboard.putNumber("Drive Speed", .25);
    autonomousStartTime = Timer.getFPGATimestamp();
  }

  @Override
  public void autonomousPeriodic() {
    AutonomousHandler.handleAutonomous(m_autoSelected, drive, autonomousStartTime);
  }

  @Override
  public void teleopInit() {
    rightBack.setNeutralMode(NeutralMode.Brake);
    leftBack.setNeutralMode(NeutralMode.Brake);
    rightFront.setNeutralMode(NeutralMode.Brake);
    leftFront.setNeutralMode(NeutralMode.Brake);
    gyro.reset();
  }

  @Override
  public void teleopPeriodic() {
    LimelightControls.getLimelightControls(drive, drive_controller, gyro);
    SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());

    if (drive_controller.getRawAxis(0) == 0) {
      gyro.reset();
    }
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
