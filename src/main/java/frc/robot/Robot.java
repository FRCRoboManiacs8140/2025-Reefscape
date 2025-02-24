// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root di rectory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
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

    switch (m_autoSelected) {
      case kSecondaryAuto:
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
      case kDefaultAuto:
        if (autoTime > 0 && autoTime < 2) {
          drive.driveCartesian(-0.25, 0, 0);
        }
        break;
    }
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
    getlimelightcontrols();
    SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());

    if (drive_controller.getRawAxis(0) == 0) {
      gyro.reset();
    }
  }

  public void getlimelightcontrols() {
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
    PIDController turnController = new PIDController(.02, .1, 0);
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

    second = (lastSecond + 1) / 50;

    if (drive_controller.getLeftBumper()) {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
      if (tv == 1) {
        drive.driveCartesian(travelTo, strafe, turn);
      }
    } else if (drive_controller.getRightBumper()) {
      current_angle = gyro.getAngle();
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
      if (tv == 1) {
        try {
          drive.driveCartesian(0, MathUtil.clamp(strafeController.calculate(tx, 0), -0.5, 0.8), botpose[5] * .0005);
        } catch (Exception e) {
          drive.driveCartesian(drive_controller.getLeftY() * movement_sensetivity, -drive_controller.getLeftX() * movement_sensetivity, -drive_controller.getRightX() * turn_sensetivity, gyroangle);
        }
      }
    } else if (drive_controller.getAButton()) {
      current_angle = gyro.getAngle();
      drive.driveCartesian(-.05, -0.3, anglePreserve.calculate(gyro.getRate(), 0), Rotation2d.fromDegrees(0));
    } else if (drive_controller.getYButton()) {
      current_angle = gyro.getAngle();
      drive.driveCartesian(-.05, 0.3, anglePreserve.calculate(gyro.getRate(), 0), Rotation2d.fromDegrees(0));
    } else if (drive_controller.getBButton()) {
      if (tv == 1) {
        drive.driveCartesian(travelToController.calculate(ta, movePoint) * -1, 0, 0);
      }
    } else if (drive_controller.getXButton()) {
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
    lastSecond = second;
  }

  private double getTagAngle(int id) {
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
