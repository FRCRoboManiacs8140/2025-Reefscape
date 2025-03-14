// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root di rectory of this project.

package frc.robot;

import java.lang.reflect.Array;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.hal.simulation.DriverStationDataJNI;
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
import edu.wpi.first.wpilibj.AnalogGyro;

//  * The VM is configured to automatically run this class, and to call the
//  * functions corresponding to
//  * each mode, as described in the TimedRobot documentation. If you change the
//  * name of this class or
//  * the package after creating this project, you must also update the
//  * build.gradle file in the
//  * project.
//  *
public class Robot extends TimedRobot {
  private static final String kFrontAuto = "Front Auto";
  private static final String kLeftAuto = "Left Auto";
  private static final String kRightAuto = "Right Auto";
  private static final String kPushRobot = "Push Robot Auto";
  private static final String kjustMoveForward = "Just Move Forward";

  private static final String levelOne = "L1";
  private static final String levelTwo = "L2";
  private static final String levelThree = "L3";

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private String m_levelSelected;
  private final SendableChooser<String> m_levelChooser = new SendableChooser<>();

  // Define motors
  private final SparkMax leftFront = new SparkMax(6, MotorType.kBrushless);
  private final SparkMax rightFront = new SparkMax(1, MotorType.kBrushless);
  private final SparkMax leftBack = new SparkMax(4, MotorType.kBrushless);
  private final SparkMax rightBack = new SparkMax(2, MotorType.kBrushless);

  private final SparkMax elevatorRight = new SparkMax(3, MotorType.kBrushless);
  private final SparkMax elevatorLeft = new SparkMax(5, MotorType.kBrushless);

  private final SparkMax endEffectorLeft = new SparkMax(7, MotorType.kBrushless);
  private final SparkMax endEffectorRight = new SparkMax(8, MotorType.kBrushless);

  // private final MecanumDrive drive = new MecanumDrive(leftFront, leftBack,
  // rightFront, rightBack);
  private final MecanumDrive drive = new MecanumDrive(leftFront, leftBack, rightFront, rightBack);

  private final XboxController drive_controller = new XboxController(0);
  private final XboxController opController = new XboxController(1);

  // private final ADIS16470_IMU gyro = new ADIS16470_IMU();
  private final AnalogGyro gyro = new AnalogGyro(0);

  // private final for encoder
  private final RelativeEncoder elevator_encoder = elevatorRight.getEncoder();

  public double autonomousStartTime, timeElapsed;
  public double strafeStartTime = 0;
  public double second = 1;
  public double lastSecond = 0;
  public double autoElevatorHeight = 60;

  double L1Position = 30;
  double L2Position = 40;
  double L3Position = 60;

  public static double getTagAngle(int id) {
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


  // CameraServer server;

  double current_angle;

  // * This function is run when the robot is first started up and should be used
  // * for any
  // * initialization code.
  // *
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Front Auto", kFrontAuto);
    m_chooser.addOption("Left Auto", kLeftAuto);
    m_chooser.addOption("Right Auto", kRightAuto);
    m_chooser.addOption("Push Robot", kPushRobot);
    m_chooser.addOption("Just Move Forward", kjustMoveForward);

    m_levelChooser.setDefaultOption("L3", levelThree);
    m_levelChooser.addOption("L2", levelTwo);
    m_levelChooser.addOption("L1", levelOne);
    

    SmartDashboard.putData("Auto choices", m_chooser);
    SmartDashboard.putData("Auto score level", m_levelChooser);
    SmartDashboard.putNumber("drive_reduction", 1);

    CameraServer.startAutomaticCapture();
    // PID for travel to autonomous movement
    SmartDashboard.putNumber("travel_to_integral_PID", 0.01);
    SmartDashboard.putNumber("travel_to_proportional_PID", 0.06);
    SmartDashboard.putNumber("travel_to_derivative_PID", 0.05);
    // PID for strafe to autonomous movement
    SmartDashboard.putNumber("strafe_to_integral_PID", 1);
    SmartDashboard.putNumber("strafe_to_proportional_PID", 0.02);
    SmartDashboard.putNumber("strafe_to_derivative_PID", 0.5);

  }

  // * This function is called every 20 ms, no matter the mode. Use this for items
  // * like diagnostics
  // * that you want ran during disabled, autonomous, teleoperated and test.
  // *
  // * <p>
  // * This runs after the mode specific periodic functions, but before LiveWindow
  // * and
  // * SmartDashboard integrated updating.
  // *
  @Override
  public void robotPeriodic() {
    // Put the current angle and game time on the dashboard
    SmartDashboard.putNumber("current angle", current_angle);
    double gameTime = Timer.getFPGATimestamp() - autonomousStartTime;
    double strafeTime = Timer.getFPGATimestamp() - strafeStartTime;
    SmartDashboard.putNumber("Strafe Time", strafeTime);
    SmartDashboard.putNumber("Time Remaining", (150 - gameTime));
    SmartDashboard.putNumber("Time elapsed", gameTime);

  }

  // * This autonomous (along with the chooser code above) shows how to select
  // * between different
  // * autonomous modes using the dashboard. The sendable chooser code works with
  // * the Java
  // * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
  // * chooser code and
  // * uncomment the getString line to get the auto name from the text box below
  // the
  // * Gyro
  // *
  // * <p>
  // * You can add additional auto modes by adding additional comparisons to the
  // * switch structure
  // * below with additional strings. If using the SendableChooser make sure to
  // add
  // * them to the
  // * chooser code above as well.
  // *
  @Override
  public void autonomousInit() {
    // Select which autonomous routine
    m_autoSelected = m_chooser.getSelected();
    m_levelSelected = m_levelChooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
    SmartDashboard.putNumber("Drive Speed", .25);

    autonomousStartTime = Timer.getFPGATimestamp();
  }

  // This function is called periodically during autonomous. *
  @Override

  public void autonomousPeriodic() {

    // Pls don't delete the code below it is not a copy
    // This is to allow PID to work during autonomous.

    // Boolean that is 1 if a target is detected, 0 if not
    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    // X angle distance from center of camera frame to center of target
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    // Y angle distance from center of camera frame to center of target
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    // Area of the camera frame that the object takes up, can be used to estimate
    // how close the object is
    double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    double botpose[] = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose")
        .getDoubleArray(new double[6]);
    double autoTime = SmartDashboard.getNumber("Time elapsed", 0);
    // values for travel to; PID
    double tkI = SmartDashboard.getNumber("travel_to_integral_PID", 0.);
    double tkP = SmartDashboard.getNumber("travel_to_proportional_PID", 0);
    double tkD = SmartDashboard.getNumber("travel_to_derivative_PID", 0);
    // values for strafe; PID
    double kI = SmartDashboard.getNumber("strafe_to_integral_PID", 0);
    double kP = SmartDashboard.getNumber("strafe_to_proportional_PID", 0);
    double kD = SmartDashboard.getNumber("strafe_to_derivative_PID", 0);
    PIDController travelToController = new PIDController(tkP, tkI, tkD);
    travelToController.setIntegratorRange(-5, 5);
    PIDController strafeController = new PIDController(kP, kI, kD);
    strafeController.setIntegratorRange(-5, 5);
    strafeController.setIZone(1);

    PIDController anglePreserve = new PIDController(.01, .1, 0);
    PIDController elevatorPID = new PIDController(0.05, 0, 0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);

    switch (m_levelSelected) {
      case levelOne:
        autoElevatorHeight = L1Position;
        break;
    
      case levelTwo:
        autoElevatorHeight = L2Position;
        break;

        case levelThree:
        autoElevatorHeight = L3Position;
        break;
    }

    switch (m_autoSelected) {
      // If Left Auto is selected . . .
      case kLeftAuto:
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(2);
        if (autoTime > 0 && autoTime < 5) {
          drive.driveCartesian(-.3, tx*-.025, anglePreserve.calculate(gyro.getRate(), 0));
        } else if (autoTime > 5 && autoTime < 8) {
          elevatorLeft.set(elevatorPID.calculate(elevator_encoder.getPosition(), autoElevatorHeight));
          elevatorRight.set(elevatorPID.calculate(elevator_encoder.getPosition(), autoElevatorHeight));
        } else if (autoTime > 8 && autoTime < 10) {
          endEffectorLeft.set(.5);
          endEffectorRight.set(-.5);
        } else if (autoTime > 10) {
          // Stop moving if time becomes more than 15
          drive.driveCartesian(0, 0, 0);
          // Needs code that switches mode to Teleop when time is over 15 seconds.
        }
        break;

      // If Front Auto is selected . . .
      case kFrontAuto:
        if (autoTime > 0 && autoTime < 2) {
          // Drive Forward for 2 seconds at 25% speed
          drive.driveCartesian(-0.25, 0, 0);
        } else if (autoTime > 2 && autoTime < 5){
          // Eject coral onto L1
          endEffectorLeft.set(.6);
          endEffectorRight.set(-.3);
        }
        break;

      case kPushRobot:
       // Move forward for 4 seconds at higher speed
       if (autoTime > 0 && autoTime < 2){
        drive.driveCartesian(-0.5,0,0);
       }

       break;

      // If right Auto is selected . . .
      case kRightAuto:
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(3);
      if (autoTime > 0 && autoTime < 5) {
        drive.driveCartesian(-.3, tx*-.025, anglePreserve.calculate(gyro.getRate(), 0));
      } else if (autoTime > 5 && autoTime < 8) {
        elevatorLeft.set(elevatorPID.calculate(elevator_encoder.getPosition(), autoElevatorHeight));
        elevatorRight.set(elevatorPID.calculate(elevator_encoder.getPosition(), autoElevatorHeight));
      } else if (autoTime > 8 && autoTime < 10) {
        endEffectorLeft.set(.5);
        endEffectorRight.set(-.5);
      } else if (autoTime > 10) {
        // Stop moving if time becomes more than 15
        drive.driveCartesian(0, 0, 0);
        // Needs code that switches mode to Teleop when time is over 15 seconds.
      }
      break;

      case kjustMoveForward:
        // Move forward for 4 seconds at higher speed
        if (autoTime > 0 && autoTime < 2){
         drive.driveCartesian(-0.3,0,0);
        }
 
        break;
    }
  }

  // This function is called once when teleop is enabled. *
  @Override
  public void teleopInit() {
    gyro.reset();
    elevator_encoder.equals(0);
    rightBack.set(0);
    leftBack.set(0);
    rightFront.set(0);
    leftFront.set(0);

  }

  // This function is called periodically during operator control. *
  @Override
  public void teleopPeriodic() {
    getlimelightcontrols();
    SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
    if (drive_controller.getAButton()) {
      gyro.reset();
    }
    PIDController elevatorPID = new PIDController(0.05, 0, 0);
    SmartDashboard.putNumber("Elevator Position", elevator_encoder.getPosition());
    // double elevator_encoder_teleop = SmartDashboard.getNumber("Elevator
    // Position", elevator_encoder.getPosition());
    // Y makes elevator go up manually
    if (opController.getYButton()) {
      elevatorLeft.set(0.1);
      elevatorRight.set(0.1);
      // A makes elevator go up manually
    } else if (opController.getAButton()) {
      elevatorLeft.set(-0.1);
      elevatorRight.set(-0.1);

      // IMPORTANT setpoints for opController levels need to be set and tuned!

      // Right Bumper is L1
    } else if (opController.getAButton()) {
      elevatorLeft.set(elevatorPID.calculate(elevator_encoder.getPosition(), L1Position));
      elevatorRight.set(elevatorPID.calculate(elevator_encoder.getPosition(), L1Position));
      // Left Bummper is L2
    } else if (opController.getXButton()) {
      elevatorLeft.set(elevatorPID.calculate(elevator_encoder.getPosition(), L2Position));
      elevatorRight.set(elevatorPID.calculate(elevator_encoder.getPosition(), L2Position));
      // Right Trigger is L3
    } else if (opController.getYButton()) {
      elevatorLeft.set(elevatorPID.calculate(elevator_encoder.getPosition(), L3Position));
      elevatorRight.set(elevatorPID.calculate(elevator_encoder.getPosition(), L3Position));
      // Left Trigger is L4
    // } else if (opController.getBButton()) {
    //   elevatorLeft.set(elevatorPID.calculate(elevator_encoder.getPosition(), 100));
    //   elevatorRight.set(elevatorPID.calculate(elevator_encoder.getPosition(), 100));
    } else {
      // Stop the elevator from moving if no buttons are being held
      elevatorLeft.set(0);
      elevatorLeft.set(0);
    }

    // Code for End Effector
    // If X is held intake coral
    if (opController.getLeftTriggerAxis() > 0.2){
      endEffectorLeft.set(-.07);
      endEffectorRight.set(.07);
    // Left Bumper shoots for L1
    } else if(opController.getLeftBumperButton()){
      endEffectorLeft.set(.6);
      endEffectorRight.set(-.3);
    // Right Bumper is to eject coral
    }else if (opController.getRightBumperButton()){
      endEffectorLeft.set(.5);
      endEffectorRight.set(-.5);
    // If Left Stick is pressed eject coral
    } else if (opController.getRightTriggerAxis() > 0.2){
      endEffectorLeft.set(-.5);
      endEffectorRight.set(.5);
    // If nothing is held the End Effector does not spin
    } else {
      endEffectorLeft.set(0);
      endEffectorRight.set(.0);
    }
  }

  // Runs the limelight function
  public void getlimelightcontrols() {

    // Outputs limelight as variables and puts them in the dashboard

    // Boolean that is 1 if a target is detected, 0 if not
    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    // X angle distance from center of camera frame to center of target
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    // Y angle distance from center of camera frame to center of target
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    // Area of the camera frame that the object takes up, can be used to estimate
    // how close the object is
    double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    double botpose[] = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose")
        .getDoubleArray(new double[6]);
    double campose[] = NetworkTableInstance.getDefault().getTable("limelight").getEntry("campose")
        .getDoubleArray(new double[6]);
    int id = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getNumber(0).intValue();
    double tagAngle = getTagAngle(id);
    

    // values for travel to; PID
    double tkI = SmartDashboard.getNumber("travel_to_integral_PID", 0.);
    double tkP = SmartDashboard.getNumber("travel_to_proportional_PID", 0.005);
    double tkD = SmartDashboard.getNumber("travel_to_derivative_PID", 0);

    // values for strafe; PID
    double kI = SmartDashboard.getNumber("strafe_to_integral_PID", .5);
    double kP = SmartDashboard.getNumber("strafe_to_proportional_PID", 0.5);
    double kD = SmartDashboard.getNumber("strafe_to_derivative_PID", 0.4);

    // Set up PID controll for Travel To motion
    PIDController travelToController = new PIDController(tkP, tkI, tkD);
    travelToController.setIntegratorRange(-5, 5);
    // Set up PID controll for Strafe To motion
    PIDController strafeController = new PIDController(kP, kI, kD);
    strafeController.setIntegratorRange(-5, 5);

    strafeController.setIZone(1);
    PIDController turnController = new PIDController(.02, .1, 0);
    PIDController anglePreserve = new PIDController(.01, .1, 0);
    SmartDashboard.putNumber("Target X", tx); // Distance between of the crosshair and the object in the X coordinate
    SmartDashboard.putNumber("Target y", ty); // Distance between of the crosshair and the object in the Y coordinate
    SmartDashboard.putNumber("Target Area", ta); // The area that the object takes up
    SmartDashboard.putNumber("Target Present", tv);

    try {
      SmartDashboard.putNumber("Rotation", botpose[5]);
    } catch (Exception e) {
    }
    // SmartDashboard.putNumber("botpose", botpose[5]);
    // Multiplier that turns the X angle distance into a motor output percent.
    // (angle is between -27 and 27, this turns it into a number between -.81 and
    // .81 that is higher the farther away the object is)
    // This needs to be tuned to the specific object and tested
    // Values for porportional drive, algea
    double turn = tx * -.005;
    double strafe = tx * -.025;
    double setPoint = 2;

    // what % of the limelight vision the april tag takes up
    double movePoint = 5;
    double travelTo = (4 - ta) * (-0.01 / ta);
    SmartDashboard.putNumber("strafe value", strafeController.calculate(tx, 0));
    SmartDashboard.putNumber("travel value", travelToController.calculate(ta, movePoint));
    // double strafe = (ty/tx)*-0.1;
    SmartDashboard.putNumber("locate", turn);

    // Multiplier that turns the area that the object takes up into a motor output
    // percent.
    SmartDashboard.putNumber("travel to", travelTo);
    // automatically drives to the object
    Rotation2d gyroangle = Rotation2d.fromDegrees(-gyro.getAngle());
    double movement_sensetivity = SmartDashboard.getNumber("drive_reduction", 0.6);
    double turn_sensetivity = 1;

    second = (lastSecond + 1) / 50;

    // Defines what happens when you press LeftBumper
    if (drive_controller.getLeftBumper()) {
      // Go to color vision
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
      if (tv == 1) {
        // Drive towards nearest Algea Gamepiece
        drive.driveCartesian(travelTo, strafe, turn);
      }
      // else if (drive_controller.getLeftBumper()) {
      //   NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(2);
      //   if (tv == 1) {
      //     getTagAngle(id);
      //   drive.driveCartesian(0, 0, turnController.calculate(gyro.getAngle() % 360, tagAngle));
      //   }
      //   if (Math.abs(gyro.getAngle() - tagAngle) < .5){
      //     drive.driveCartesian(0, strafeController.calculate(tx, 0), anglePreserve.calculate(gyro.getAngle(), 0));
      //   }
      //   if (tx < .5) {
      //     drive.driveCartesian(.3, 0, anglePreserve.calculate(gyro.getAngle(), 0));
      //   }
      //   }
      // Defines what happens when you press RightBumper
    } else if (drive_controller.getRightBumper()) {
      current_angle = gyro.getAngle();
      // Go to April Tag Vision
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
      if (tv == 1) {
        // Go to nearest April Tag
        // Commented out to test new code!!!
        // drive.driveCartesian(travelTo, strafe, ((-(gyro.getAngle() - current_angle) %
        // 360) * .5),
        // Rotation2d.fromDegrees(0));
        try {

          drive.driveCartesian(
              0, // MathUtil.clamp((travelToController.calculate(ta,movePoint)*-1*movement_sensetivity),-0.5,0.5),
              (MathUtil.clamp(strafeController.calculate(tx, 0), -0.8, 0.8)),
              campose[4] * .0005);

        } catch (Exception e) {
          drive.driveCartesian(
              drive_controller.getLeftY() * movement_sensetivity,
              -drive_controller.getLeftX() * movement_sensetivity,
              drive_controller.getRightX() * turn_sensetivity, gyroangle);
        }
        // if (drive_controller.get)
      }

      // according to all known laws of avation there should be no way a bee is able
      // to fly its wings are too small to get its fat little body off of the ground
      // the bee of course flys

      // Defines what happens when you press AButton
      // } else if (drive_controller.getXButton()) {
      // // Strafe left
      // current_angle = gyro.getAngle();
      // drive.driveCartesian(0, 0.23, ((-(gyro.getAngle() - current_angle) % 360) *
      // .05), Rotation2d.fromDegrees(0));
    } else if (drive_controller.getBButton()) {
      // Strafe right
      current_angle = gyro.getAngle();
      drive.driveCartesian(-.05, 0.3, (anglePreserve.calculate(gyro.getRate(), 0)), Rotation2d.fromDegrees(0));
      // Defines what happens when you press BButton
    } else if (drive_controller.getYButton()) {
      if (tv == 1) {
        // Moves forward using travelTo PID Controller
        drive.driveCartesian(travelToController.calculate(ta, movePoint) * -1, 0, 0);
      }
      // Defines what happens when you press Right Trigger
    } else if (drive_controller.getRightTriggerAxis() >= 0.2) {
      // Set the limelight to the right offset of the April tag
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(3);
      // Check to see if the robot can see an April tag
      if (tv == 1) {
        try {
          drive.driveCartesian(
              // Strafe to the April tag
              0,
              MathUtil.clamp(strafeController.calculate(tx, 0), -0.5, 0.8),
              botpose[5] * .0005);
          // Set robot to manual controll if the robot can't see a April tag
        } catch (Exception e) {
          drive.driveCartesian(
              -drive_controller.getLeftY() * movement_sensetivity,
              drive_controller.getLeftX() * movement_sensetivity,
              drive_controller.getRightX() * turn_sensetivity, gyroangle);
        }
      }
    } else if (drive_controller.getLeftTriggerAxis() >= 0.2) {
      // When you hold the Left Trigger
      // Set the limelight to the left offset of the April tag
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(2);
      // Check to see if the robot can see an April tag
      if (tv == 1) {
        try {
          // Strafe to the April tag
          drive.driveCartesian(
              0,
              MathUtil.clamp(strafeController.calculate(tx, 0), -0.5, 0.8),
              botpose[5] * .0005);
          // Set robot to manual control if the robot can't see a April tag
        } catch (Exception e) {
          drive.driveCartesian(
              -drive_controller.getLeftY() * movement_sensetivity,
              drive_controller.getLeftX() * movement_sensetivity,
              drive_controller.getRightX() * turn_sensetivity, gyroangle);
        }
      }
    } else if (drive_controller.getXButton()) {
      drive.driveCartesian(-.05, -0.3, (anglePreserve.calculate(gyro.getRate(), 0)), Rotation2d.fromDegrees(0));
    } else {
      // Set robot to manual control if the robot can't see a April tag(This is NOT a
      // duplicate plz don't delete)
      drive.driveCartesian(
          -drive_controller.getLeftY() * movement_sensetivity * movement_sensetivity,
          drive_controller.getLeftX() * movement_sensetivity * movement_sensetivity,
          drive_controller.getRightX() * turn_sensetivity, gyroangle);
    }

  } // ends teleop

  // This function is called once when the robot is disabled. *
  @Override
  public void disabledInit() {
  }

  // This function is called periodically when disabled. *
  @Override
  public void disabledPeriodic() {
  }

  // This function is called once when test mode is enabled. *
  @Override
  public void testInit() {
  }

  // This function is called periodically during test mode. *
  @Override
  public void testPeriodic() {
  }

  // This function is called once when the robot is first started up. *
  @Override
  public void simulationInit() {
  }

  // This function is called periodically whilst in simulation. *
  @Override
  public void simulationPeriodic() {
  }
}
