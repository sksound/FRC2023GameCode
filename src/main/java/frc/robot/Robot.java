// TEST
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer.autoBalance;
import frc.robot.subsystems.DriveSubsystem;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Set;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy; 


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  //global variables

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  
  
  Joystick stick;
  Joystick stick2;

  double swerveSpeed;
  double rotateSpeed;

  double armMotorSpeedUp = 1;
  double armMotorSpeedDown = 1;

  String[] autonomousList = {"PPSwerveCommand", "coneCubeCorner", "cubeAutoBalance"};

  String autoSelected;

  public  CANSparkMax arm_motor = new CANSparkMax(18, MotorType.kBrushless);
  public CANSparkMax arm_motor2 = new CANSparkMax(19, MotorType.kBrushless);
  public CANSparkMax ext_arm_motor = new CANSparkMax(50, MotorType.kBrushless);

  DutyCycleEncoder encoder = new DutyCycleEncoder(0);

  PIDController pid = new PIDController(3, 0, 2);

  double setpoint;
  double intermediate_setpoint;

  UsbCamera camera;
  UsbCamera camera2;

  NetworkTableEntry cameraSelection;


  // Pneumatics
  public static DoubleSolenoid telescopic_arm  = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 2);
 

 
 
  @Override
  public void robotInit() {

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    stick = m_robotContainer.m_driverController;

    SmartDashboard.putStringArray("Auto List", autonomousList);
    


    camera = CameraServer.startAutomaticCapture(0);
    camera.setResolution(320, 240);

    camera2 = CameraServer.startAutomaticCapture(1);
     camera2.setResolution(320, 240);

    
    cameraSelection = NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection");
    // cameraSelection.setString(camera.getName());
    
    camera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    camera2.setConnectionStrategy(ConnectionStrategy.kKeepOpen);


    cameraSelection.setString(camera.getName());


    stick2 = new Joystick(1);

    setpoint = .5;
    m_robotContainer.armState = setpoint;
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    setpoint = .5;

    autoSelected = SmartDashboard.getString("Auto Selector", "None");
    
    switch(autoSelected) {
      case "superCoolAuto":
      m_autonomousCommand = m_robotContainer.getAutonomousCommand();
      break;
      case "autoBalanceTest":
      m_autonomousCommand = m_robotContainer.newCommand();
      break;
      case "armTest":
      m_autonomousCommand = m_robotContainer.armStuff();
      break;
      case "simpleCubeAuto":
      m_autonomousCommand = m_robotContainer.simpleCubeAuto();
      break;
      case "PPSwerveCommand":
      m_autonomousCommand = m_robotContainer.followTrajectoryCommand();
      break;
      case "coneCubeCorner":
      m_autonomousCommand = m_robotContainer.coneCubeCorner();
      break;
      case "cubeAutoBalance":
      m_autonomousCommand = m_robotContainer.cubeAutoBalance();
      break;
    }
    
    


    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    
   
  
  }


  

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic () {


  // At the beginning of auto

  m_autonomousCommand.execute();
      // auto here  

  if(m_robotContainer.m_robotDrive.dummyBoo){
    m_robotContainer.m_robotDrive.autoBalance();
  }


  if(m_robotContainer.isOpen){
    telescopic_arm.set(Value.kReverse);
    System.out.println("Open");
    
  }
  else if(m_robotContainer.isOpen == false){
    telescopic_arm.set(Value.kForward);
    System.out.println("Close");
    
  }

  setpoint = m_robotContainer.armState;
  
  arm_motor.set(-pid.calculate(encoder.getAbsolutePosition(), setpoint));
  arm_motor2.set(-pid.calculate(encoder.getAbsolutePosition(), setpoint));

      


}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
    setpoint = .5;

  
  }



  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Slow Mode for rotation and movement
    if(stick.getRawAxis(2)>.5){
      Constants.DriveConstants.kMaxSpeedMetersPerSecond = 1;
      Constants.DriveConstants.kMaxAngularSpeed = .5 * Math.PI;
    }
    // Fast Mode
    else if(stick.getRawAxis(3)>.5){
      Constants.DriveConstants.kMaxSpeedMetersPerSecond = 5.5;
     
    }
    else{
      Constants.DriveConstants.kMaxSpeedMetersPerSecond = 3;
      Constants.DriveConstants.kMaxAngularSpeed = 1.5 * Math.PI;
    }


    
    
    if(stick2.getRawButton(1)){ 
      setpoint = 0.5;
      System.out.println(setpoint);
      }
    else if(stick2.getRawButton(2)){
      setpoint = 0.765;
      System.out.println(setpoint);
    }
    else if(stick2.getRawButton(3)){
      setpoint = 0.37;
      System.out.println(setpoint);
    }
    else if(stick2.getRawButton(4)){
      setpoint = 0.23;
      System.out.println(setpoint);
    }

    if (Math.abs(encoder.getAbsolutePosition() - setpoint) > 0.5) {
      intermediate_setpoint = (encoder.getAbsolutePosition() + setpoint) / 2;
      System.out.println("running");
    }
    else {
      intermediate_setpoint = setpoint;
    }

    
    
    arm_motor.set(-pid.calculate(encoder.getAbsolutePosition(), intermediate_setpoint));
    arm_motor2.set(-pid.calculate(encoder.getAbsolutePosition(), intermediate_setpoint));
    
    ext_arm_motor.set(0);
    if(stick.getRawButton(2)){
      ext_arm_motor.set(1);
    }
    if(stick.getRawButton(1)){
      ext_arm_motor.set(-1);
    }
    
    
    //claw open
    if(stick2.getRawButton(7)){
      telescopic_arm.set(Value.kReverse);
      System.out.println("switched reverse");
    }
    //claw closed
    if(stick2.getRawButton(8)){
      telescopic_arm.set(Value.kForward);
      System.out.println("switched forward");
    }
            
    
     if(stick.getRawButton(8)){
       DriveSubsystem.m_gyro.reset();
     }

     if (stick.getRawButton(1)){ 
      cameraSelection.setString(camera.getName());
      System.out.println("cam1 set");
      } 
    else if (stick.getRawButton(2)) {
      cameraSelection.setString(camera2.getName());
      System.out.println("cam2 set");
      }
  
  
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

    



  }
  
}
