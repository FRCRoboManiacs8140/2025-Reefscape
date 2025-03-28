// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root di rectory of this project.

package frc.robot;

import java.lang.reflect.Array;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.hal.simulation.DriverStationDataJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;

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

   private DigitalInput elevatorLimit = new DigitalInput(0); // Limit switch for elevator
   private DigitalInput intakebeambreak = new DigitalInput(1);

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

  private final Spark blinkinSpark = new Spark(3);

  // private final MecanumDrive drive = new MecanumDrive(leftFront, leftBack,
  // rightFront, rightBack);
  private final MecanumDrive drive = new MecanumDrive(leftFront, leftBack, rightFront, rightBack);

  private final XboxController drive_controller = new XboxController(0);
  private final XboxController opController = new XboxController(1);

  //private final ADIS16470_IMU gyro = new ADIS16470_IMU();
  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  // private final for encoder
  private final RelativeEncoder elevator_encoder = elevatorRight.getEncoder();

  public double autonomousStartTime, timeElapsed;
  public double strafeStartTime = 0;
  public double second = 1;
  public double lastSecond = 0;
  public double autoElevatorHeight = 58;

  double L1Position = 1;
  double L2Position = 48;
  double L3Position = 59;
  double intakePosition = 1;

  double offset = 0;

  public static double getTagAngle(int id) {
    switch (id) {
      case 7:
      case 18:
        return 0;
      case 6:
      case 19:
        return 300;
      case 11:
      case 20:
        return 240;
      case 10:
      case 21:
        return 180;
      case 9:
      case 22:
        return 120;
      case 8:
      case 17:
        return 60;
      default:
        return 0;
    }
}


  // CameraServer server;

  // * This function is run when the robot is first started up and should be used
  // * for any
  // * initialization code.
  // *
  @Override
  public void robotInit() {
    gyro.calibrate();
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
    SmartDashboard.putNumber("turn_reduction", 1);

    new Thread(() -> {
    //UsbCamera camera = 
    CameraServer.startAutomaticCapture();
    CvSink cvSink = CameraServer.getVideo();
    CvSource outputStream = CameraServer.putVideo("Intake Camera with box", 320, 240);
    outputStream.setFPS(15);
    outputStream.setResolution(320,240);

    Mat frame = new Mat();
    Point pt1 = new Point(120, 80);
    Point pt2 = new Point(120, 160);
    Point pt3 = new Point(200, 80);
    Point pt4 = new Point(200, 160);
    Scalar color = new Scalar(0, 255, 0);
                
    while(!Thread.interrupted()) {
      if (cvSink.grabFrame(frame) == 0) {
        continue;
      }
      Imgproc.line(frame, pt1, pt2, color, 3);
      Imgproc.line(frame, pt2, pt4, color, 3);
      Imgproc.line(frame, pt4, pt3, color, 3);
      Imgproc.line(frame, pt3, pt1, color, 3);
      outputStream.putFrame(frame);
    }
  }).start();
    // PID for travel to autonomous movement
    SmartDashboard.putNumber("travel_to_integral_PID", 0.01);
    SmartDashboard.putNumber("travel_to_proportional_PID", 0.06);
    SmartDashboard.putNumber("travel_to_derivative_PID", 0.05);
    // PID for strafe to autonomous movement
    SmartDashboard.putNumber("strafe_to_integral_PID", .0);
    SmartDashboard.putNumber("strafe_to_proportional_PID", .02);
    SmartDashboard.putNumber("strafe_to_derivative_PID", 0);

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
   
  System.out.println("Intake Beam Break: " + intakebeambreak.get());

    Rotation2d gyroangle = Rotation2d.fromDegrees(-gyro.getAngle() + offset);
    SmartDashboard.putNumber("gyro angle", gyroangle.getDegrees());
    SmartDashboard.putNumber("gyro rate", gyro.getRate()*-1);
    SmartDashboard.putNumber("current angle", gyroangle.getDegrees() % 360);
    double gameTime = Timer.getFPGATimestamp() - autonomousStartTime;
    double strafeTime = Timer.getFPGATimestamp() - strafeStartTime;
    SmartDashboard.putNumber("Strafe Time", strafeTime);
    SmartDashboard.putNumber("Time Remaining", (150 - gameTime));
    SmartDashboard.putNumber("Time elapsed", gameTime);
    SmartDashboard.putBoolean("Beam Break Override", false);
    SmartDashboard.putData("beam break", intakebeambreak);
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
    int id = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getNumber(0).intValue();
    double autoTime = SmartDashboard.getNumber("Time elapsed", 0);
    // values for travel to; PID
    double tkI = SmartDashboard.getNumber("travel_to_integral_PID", 0);
    double tkP = SmartDashboard.getNumber("travel_to_proportional_PID", 0);
    double tkD = SmartDashboard.getNumber("travel_to_derivative_PID", 0);
    // values for strafe; PID
    double kI = SmartDashboard.getNumber("strafe_to_integral_PID", 0);
    double kP = SmartDashboard.getNumber("strafe_to_proportional_PID", .25);
    double kD = SmartDashboard.getNumber("strafe_to_derivative_PID", 0);
    PIDController travelToController = new PIDController(tkP, tkI, tkD);
    travelToController.setIntegratorRange(-5, 5);
    PIDController strafeController = new PIDController(kP, kI, kD);
    strafeController.setIntegratorRange(-5, 5);
    strafeController.setIZone(1);
    PIDController turnController = new PIDController(.02, .1, 0);
    double tagAngle = getTagAngle(id);

    PIDController anglePreserve = new PIDController(.01, 0, 0);
    PIDController elevatorPID = new PIDController(0.08, 0.1, 0);
    elevatorPID.setIntegratorRange(-5, 5);
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
          drive.driveCartesian(.2, -MathUtil.clamp(strafeController.calculate(tx, 0), (-(33-ta)/100), ((33-ta)/100)), turnController.calculate((gyro.getAngle() + 3600) % 360, tagAngle));
        } else if (autoTime > 5 && autoTime < 8) {
          elevatorLeft.set(elevatorPID.calculate(elevator_encoder.getPosition(), autoElevatorHeight)*.5);
          elevatorRight.set(elevatorPID.calculate(elevator_encoder.getPosition(), autoElevatorHeight)*.5);
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
        if (autoTime > 0 && autoTime < 5) {
          // Drive Forward for 2 seconds at 25% speed
          drive.driveCartesian(0.25, 0, anglePreserve.calculate(gyro.getRate(), 0));
        } else if (autoTime > 5 && autoTime < 7){
          // Eject coral onto L1
          endEffectorLeft.set(.6);
          endEffectorRight.set(-.3);
        }
        break;

      case kPushRobot:
       // Move forward for 4 seconds at higher speed
       if (autoTime > 0 && autoTime < 2){
        drive.driveCartesian(.95,0,0);
       }

       break;

      // If right Auto is selected . . .
      case kRightAuto:
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(3);
      if (autoTime > 0 && autoTime < 5) {
        drive.driveCartesian(.2, -MathUtil.clamp(strafeController.calculate(tx, 0), (-(33-ta)/100), ((33-ta)/100)), turnController.calculate((gyro.getAngle() + 3600) % 360, tagAngle));
      } else if (autoTime > 5 && autoTime < 8) {
        elevatorLeft.set(elevatorPID.calculate(elevator_encoder.getPosition(), autoElevatorHeight)*.5);
        elevatorRight.set(elevatorPID.calculate(elevator_encoder.getPosition(), autoElevatorHeight)*.5);
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
         drive.driveCartesian(0.3,0,0);
        }
 
        break;
    }
  }

  // This function is called once when teleop is enabled. *
  @Override
  public void teleopInit() {
    //elevator_encoder.equals(0);
    rightBack.set(0);
    leftBack.set(0);
    rightFront.set(0);
    leftFront.set(0);
  }

  // This function is called periodically during operator control. *
  @Override
  public void teleopPeriodic() {
    getlimelightcontrols();
    controlElevator(); // Refactored elevator control logic into a separate method

    // Code for End Effector

    if (opController.getLeftTriggerAxis() > 0.2){
      endEffectorLeft.set(.07);
      endEffectorRight.set(-.07);
      
    // Left Bumper shoots for L1
    } else if(opController.getLeftBumperButton()){
      endEffectorLeft.set(.6);
      endEffectorRight.set(-.3);
    // Right Bumper is to eject coral
     }else if (opController.getRightBumperButton()){
      endEffectorLeft.set(-.05);
      endEffectorRight.set(.05);
    // If Right Trigger is pressed eject coral 
    } else if (opController.getRightTriggerAxis() > 0.2){
      endEffectorLeft.set(.5);
      endEffectorRight.set(-.5);
    // If the beam break is tripped, the intake automatically spins until it is cleared
    } else if (!intakebeambreak.get()) {
      endEffectorLeft.set(.07);
      endEffectorRight.set(-.07);
      opController.setRumble(GenericHID.RumbleType.kLeftRumble,.5);
      opController.setRumble(GenericHID.RumbleType.kRightRumble,.5);
      // If nothing is held the End Effector does not spin
    } else {
      endEffectorLeft.set(0);
      endEffectorRight.set(0);
      opController.setRumble(GenericHID.RumbleType.kLeftRumble,0);
      opController.setRumble(GenericHID.RumbleType.kRightRumble,0);
    }
  }
    
  

  private void controlElevator() {
    PIDController elevatorPID = new PIDController(0.05, 0, 0);
    PIDController elevatorbottomPID = new PIDController(0.025, 0, 0);
    SmartDashboard.putNumber("Elevator Position", elevator_encoder.getPosition());
    SmartDashboard.putBoolean("Elevator Limit", elevatorLimit.get());

    if (!elevatorLimit.get()) {
        elevator_encoder.setPosition(0);
    }

    if (opController.getPOV() == 0) {
        elevatorLeft.set(0.1);
        elevatorRight.set(0.1);
    } else if (opController.getPOV() == 180) {
        elevatorLeft.set(-0.1);
        elevatorRight.set(-0.1);
        if (!elevatorLimit.get() && elevatorRight.get() < 0) {
            elevatorLeft.set(0);
            elevatorRight.set(0);
        }
    } else if (opController.getAButton()) {
        setElevatorPosition(elevatorPID, L1Position);
        if (!elevatorLimit.get() && elevatorRight.get() < 0) {
          elevatorLeft.set(0);
          elevatorRight.set(0);
        }
    } else if (opController.getXButton()) {
        setElevatorPosition(elevatorPID, L2Position);
    } else if (opController.getYButton()) {
        setElevatorPosition(elevatorPID, L3Position);
    } else if (opController.getBButton()) {
        setElevatorPosition(elevatorbottomPID, intakePosition);
        if (!elevatorLimit.get() && elevatorRight.get() < 0) {
            elevatorLeft.set(0);
            elevatorRight.set(0);
        }
    } else {
        elevatorLeft.set(0);
        elevatorRight.set(0);
    }
}

private void setElevatorPosition(PIDController pidController, double targetPosition) {
    double output = pidController.calculate(elevator_encoder.getPosition(), targetPosition) * 0.5;
    elevatorLeft.set(output);
    elevatorRight.set(output);
}

  // Runs the limelight function
  public void getlimelightcontrols() {

    if (drive_controller.getAButton()) {
      offset = 0;
      gyro.reset();
    }
    if (drive_controller.getXButton()) {
      offset = 306;
      gyro.reset();
    }
    if (drive_controller.getBButton()) {
      offset = 54;
      gyro.reset();
    }
    Rotation2d gyroangle = Rotation2d.fromDegrees((gyro.getAngle()) + offset);

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

    int id = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getNumber(0).intValue();
    double tagAngle = getTagAngle(id);
    SmartDashboard.putNumber("Tag Angle", tagAngle);
    SmartDashboard.putNumber("Tag ID", id);
    

    // values for travel to; PID
    double tkI = SmartDashboard.getNumber("travel_to_integral_PID", 0.);
    double tkP = SmartDashboard.getNumber("travel_to_proportional_PID", 0.005);
    double tkD = SmartDashboard.getNumber("travel_to_derivative_PID", 0);

    // values for strafe; PID
    double kI = SmartDashboard.getNumber("strafe_to_integral_PID", .0);
    double kP = SmartDashboard.getNumber("strafe_to_proportional_PID", .025);
    double kD = SmartDashboard.getNumber("strafe_to_derivative_PID", 0);

    // Set up PID controll for Travel To motion
    PIDController travelToController = new PIDController(tkP, tkI, tkD);
    travelToController.setIntegratorRange(-5, 5);
    // Set up PID controll for Strafe To motion
    PIDController strafeController = new PIDController(kP, kI, kD);
    strafeController.setIntegratorRange(-5, 5);

    strafeController.setIZone(1);
    PIDController turnController = new PIDController(.02, .1, 0);
    PIDController anglePreserve = new PIDController(.01, 0, 0);
    SmartDashboard.putNumber("Target X", tx); // Distance between of the crosshair and the object in the X coordinate
    SmartDashboard.putNumber("Target y", ty); // Distance between of the crosshair and the object in the Y coordinate
    SmartDashboard.putNumber("Target Area", ta); // The area that the object takes up
    SmartDashboard.putNumber("Target Present", tv);
    SmartDashboard.putNumber("Target ID", id);

    // what % of the limelight vision the april tag takes up
    double movePoint = 5;
    double travelTo = (4 - ta) * (-0.01 / ta);
    SmartDashboard.putNumber("strafe value", strafeController.calculate(tx, 0));
    SmartDashboard.putNumber("travel value", travelToController.calculate(ta, movePoint));

    // Multiplier that turns the area that the object takes up into a motor output
    // percent.
    SmartDashboard.putNumber("travel to", travelTo);
    // automatically drives to the object
    double movement_sensetivity = SmartDashboard.getNumber("drive_reduction", 1);
    double turn_sensetivity = SmartDashboard.getNumber("turn_reduction", 1);

     
    
  /// DRIVE BUTTONS ________________________________________________________________________________________________
    
    ///Auto Aligning-----------------------------------------------
  
  /// Combined Aligning----------  
    if (drive_controller.getLeftBumper()) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(2);
        
        if (tv == 1) {
          drive.driveCartesian(drive_controller.getLeftY()*-.2, -MathUtil.clamp(strafeController.calculate(tx, 0), (-(33-ta)/100), ((33-ta)/100)), 0/*MathUtil.clamp(turnController.calculate((gyro.getAngle() + 3600 + offset) % 360, tagAngle),-.5,.5)*/);
        //   getTagAngle(id);
        // drive.driveCartesian(0, 0, turnController.calculate(gyro.getAngle() % 360, tagAngle));
        // }
        // if (Math.abs(gyro.getAngle() - tagAngle) < 1){
        //   drive.driveCartesian(0, -MathUtil.clamp(strafeController.calculate(tx, 0), (-(33-ta)/100), ((33-ta)/100)), anglePreserve.calculate(gyro.getAngle(), 0));
      
        }
    } else if (drive_controller.getRightBumper()) {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(3);

      if (tv ==1) {
      drive.driveCartesian(drive_controller.getLeftY()*-.2, -MathUtil.clamp(strafeController.calculate(tx, 0), (-(33-ta)/100), ((33-ta)/100)), 0/*-MathUtil.clamp(turnController.calculate((gyro.getAngle() + offset) % 360, tagAngle),-.5,.5)*/);
      }
      // if (tv == 1) {
      //   getTagAngle(id);
      //   drive.driveCartesian(0, 0, turnController.calculate(gyro.getAngle() % 360, tagAngle));
      // }
      // if (Math.abs(gyro.getAngle() - tagAngle) < 1){
      //   drive.driveCartesian(0, MathUtil.clamp(strafeController.calculate(tx, 0), (-(33-ta)/100), ((33-ta)/100)), anglePreserve.calculate(gyro.getAngle(), 0));
      // }

  /// Strafe Only Auto Aligning ----------------------------

    } else if (drive_controller.getRightTriggerAxis() >= 0.2) {
      // Set the limelight to the right offset of the April tag
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(3);
      // Check to see if the robot can see an April tag
      if (tv == 1) {
        try {
          // Strafe to the April tag
          drive.driveCartesian(
              0,
              -MathUtil.clamp(strafeController.calculate(tx, 0), (-(33-ta)/100), ((33-ta)/100)),
              0);
          // Set robot to manual control if the robot can't see a April tag
        } catch (Exception e) {
          drive.driveCartesian(
              -drive_controller.getLeftY() * movement_sensetivity,
              drive_controller.getLeftX() * movement_sensetivity,
              drive_controller.getRightX() * turn_sensetivity,
              gyroangle);
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
              -MathUtil.clamp(strafeController.calculate(tx, 0), (-(33-ta)/100), ((33-ta)/100)),
              0);
          // Set robot to manual control if the robot can't see a April tag
        } catch (Exception e) {
          drive.driveCartesian(
              -drive_controller.getLeftY() * movement_sensetivity,
              drive_controller.getLeftX() * movement_sensetivity,
              drive_controller.getRightX() * turn_sensetivity,
              gyroangle);
        }
      }

      /// Manual Aligning-----------------------

    } else if (drive_controller.getPOV() == 90) {
      // Strafe right
      drive.driveCartesian(.05, 0.1, (anglePreserve.calculate(gyro.getRate(), 0)), Rotation2d.fromDegrees(0));

    } else if (drive_controller.getPOV() == 270) {
      drive.driveCartesian(.05, -0.1, (anglePreserve.calculate(gyro.getRate(), 0)), Rotation2d.fromDegrees(0));

    } else if (drive_controller.getPOV() == 0){
      drive.driveCartesian(.2, 0, (anglePreserve.calculate(gyro.getRate(), 0)), Rotation2d.fromDegrees(0));

    } else if (drive_controller.getYButton()) {
      if (tv == 1) {
        // Moves forward using travelTo PID Controller
        drive.driveCartesian(travelToController.calculate(ta, movePoint), 0, 0);
      }


   /// Normal Driving-----------------------
   
    } else {
      
      drive.driveCartesian(
          -drive_controller.getLeftY() * movement_sensetivity,
          drive_controller.getLeftX() * movement_sensetivity,
          drive_controller.getRightX() * turn_sensetivity,
          (gyroangle));
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
