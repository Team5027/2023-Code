// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//careers.j&j.com 


package frc.robot;

//import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.wpilibj.Encoder; 
//import edu.wpi.first.wpilibj.XboxController;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kOff;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
// camera methods
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import edu.wpi.first.networktables.NetworkTable;
// intake - pneumatics
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.DigitalInput;

//controller
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  NetworkTableEntry xEntry;
  NetworkTableEntry yEntry;

  public static final int can0 = 0;
  public static final int can1 = 1;
  public static final int can2 = 2;
  public static final int can3 = 3;
  public static final int can4 = 4;
  public static final int can5 = 5;

// controller
  Joystick joy1 = new Joystick(0); //inputs for Joystick
  Joystick joy2 = new Joystick(1);// inputs for Joystick 2

  //motors
  WPI_VictorSPX leftFrontMotor = new WPI_VictorSPX(can0);
  WPI_VictorSPX rightFrontMotor = new WPI_VictorSPX(can1);
  WPI_VictorSPX leftRearMotor = new WPI_VictorSPX(can2);
  WPI_VictorSPX rightRearMotor = new WPI_VictorSPX(can3);

  MotorController rightSide = new MotorControllerGroup(rightFrontMotor, rightRearMotor);
  MotorController leftSide = new MotorControllerGroup(leftFrontMotor, leftRearMotor);
  DifferentialDrive _drive = new DifferentialDrive(leftSide, rightSide);

// elevator
  WPI_VictorSPX elavatorMotor = new WPI_VictorSPX(can5);
  WPI_VictorSPX armMotor = new WPI_VictorSPX(can4);
  MotorController up = new MotorControllerGroup(armMotor);
  MotorController down = new MotorControllerGroup(armMotor);
  DifferentialDrive _armDrive = new DifferentialDrive(up, down);

  
//pneumatics
  Compressor Comp = new Compressor(PneumaticsModuleType.CTREPCM);
  DoubleSolenoid pincherDoublePCM = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 0);

  double kP = 0;
  double kI = 0;
  double kD = 0;
  PIDController pid = new PIDController(kP, kI, kD);
  //private ArmStateMachine armStateMachine;

  
  
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    
    pincherDoublePCM.set(kOff); 
    leftFrontMotor.setNeutralMode(NeutralMode.Coast);
    leftRearMotor.setNeutralMode(NeutralMode.Coast); 
    rightFrontMotor.setNeutralMode(NeutralMode.Coast); 
    rightRearMotor.setNeutralMode(NeutralMode.Coast);  
    
    CameraServer.startAutomaticCapture();
    CvSink cvSink = CameraServer.getVideo();

// Creates the CvSource and MjpegServer [2] and connects them
CvSource outputStream = CameraServer.putVideo("Blur", 640, 480);

    UsbCamera usbCamera = new UsbCamera("USB Camera 0", 0);
try (MjpegServer cameraServer = new MjpegServer("serve_USB Camera 0", 1181)) {
  cameraServer.setSource(usbCamera);
}
try (CvSink vSink = new CvSink("opencv_USB Camera 0")) {
  cvSink.setSource(usbCamera);
}
try (CvSource OutputStream = new CvSource("Blur", PixelFormat.kMJPEG, 640, 480, 30)) {
}
try (MjpegServer mjpegServer2 = new MjpegServer("serve_Blur", 1182)) {
  mjpegServer2.setSource(outputStream);
}


    //armStateMachine = new ArmStateMachine(armMotor);
  }

  //private MotorController newMotorControllerGroup(WPI_VictorSPX armMotor2) {
    //return null;
  //}

  @Override
  public void robotPeriodic() {}

  //private DifferentialDrive _drive;
  //private final double kSpeed = 0.25;
  //private final double kTime = 5.0;
  private double startTime;
@Override
 public void autonomousInit() {
 startTime = Timer.getFPGATimestamp();
 }
  //@Override
 // public void autonomousInit() {
  //startTime = Timer.getFPGATimestamp();
  //_drive.arcadeDrive(0.0, 0.0);
  //Timer.delay(0.5);
  //_drive.arcadeDrive(kSpeed, 0.0);
  //Timer.delay(kTime);
  //_drive.arcadeDrive(0.0, 0.0);
  //}

   @Override
  public void autonomousPeriodic() {
   Double time = Timer.getFPGATimestamp();

   leftFrontMotor.setNeutralMode(NeutralMode.Brake);
   rightFrontMotor.setNeutralMode(NeutralMode.Brake);
   leftRearMotor.setNeutralMode(NeutralMode.Brake);
  rightRearMotor.setNeutralMode(NeutralMode.Brake);
 
    if (time - startTime <= 2) {
      leftSide.set(-0.4);
      rightSide.set(0.4);
    }
 else if (time - startTime >= 4 && time - startTime <= 7){
  leftSide.set(0.3);                                              
  rightSide.set(-0.3);
}
  else {
  leftSide.set(0);
  rightSide.set(0);
 }
 
    
  }

  @Override
  public void teleopInit() {
 /* factory default values */
   leftFrontMotor.configFactoryDefault();
   rightFrontMotor.configFactoryDefault();
   leftRearMotor.configFactoryDefault();
   rightRearMotor.configFactoryDefault();
  
   leftFrontMotor.setNeutralMode(NeutralMode.Brake);
   rightFrontMotor.setNeutralMode(NeutralMode.Brake);
   leftRearMotor.setNeutralMode(NeutralMode.Brake);
  rightRearMotor.setNeutralMode(NeutralMode.Brake);



 /* flip values so robot moves forward when stick-forward/LEDs-green */
    leftSide.setInverted(true); // <<<<<< Adjust this
    rightSide.setInverted(false); // <<<<<< Adjust this

    }    

  @Override
  public void teleopPeriodic() {
    double xSpeed = joy1.getRawAxis(1);
    double zRotation = joy1.getRawAxis(2);
    _drive.arcadeDrive(xSpeed * 0.3, zRotation * 0.3);
   /* 
    if(xSpeed < 0){
      xSpeed = -xSpeed*xSpeed;
    }
    else{
      xSpeed = xSpeed*xSpeed;
    }
    if(zRotation < 0){
      zRotation = -zRotation * zRotation;
    }
    else{
      zRotation = zRotation * zRotation;
    }
    */

    _drive.arcadeDrive(xSpeed, zRotation);

    if (joy1.getRawButtonPressed(6)) {
      Comp.disable();
    } 
    else if (joy1.getRawButtonPressed(5)) {
    Comp.enableDigital();
    }

    else {
 
    }
    
    if (joy2.getRawButtonPressed(1)) {
      pincherDoublePCM.set(kForward);
    }
    if (joy2.getRawButtonPressed(3)) {
      pincherDoublePCM.set(kReverse);
    }
    armMotor.set(joy2.getRawAxis(1) * 0.7);
    elavatorMotor.set(joy2.getRawAxis(3) * 0.2);


    if(joy1.getRawButtonPressed(2)){
      leftFrontMotor.setNeutralMode(NeutralMode.Brake);
   rightFrontMotor.setNeutralMode(NeutralMode.Brake);
   leftRearMotor.setNeutralMode(NeutralMode.Brake);
  rightRearMotor.setNeutralMode(NeutralMode.Brake);
    }
else{
  leftFrontMotor.configFactoryDefault();
   rightFrontMotor.configFactoryDefault();
   leftRearMotor.configFactoryDefault();
   rightRearMotor.configFactoryDefault();
}
    /* 
    armStateMachine.periodic();

    if (joy1.getRawButton(2)) {
        armStateMachine.moveToMovingUp();
    } 
    else if (joy1.getRawButtonPressed(3)) {
        armStateMachine.moveToMovingDown();
    }
     else {
      armMotor.set(0);
        armStateMachine.moveToIdle();
    }
    */
  }

 public void arcadeArmDrive(double xArm, double zArm){

 /* armMotor.set(-0.5);
  Object armStateMachine;
  armStateMachine.moveToMovingDown();*/

  arcadeArmDrive(xArm, zArm);
   xArm = joy2.getRawAxis(1);
  // zArm = joy2.getRawAxis(3);
   armMotor.set(joy2.getRawAxis(1));

 }
 
  
    //if (joy1.getRawButton(4)){
      //elavatorMotor.set(-0.5);

  //}
     
    

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}