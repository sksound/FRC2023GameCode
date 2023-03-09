// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.io.IOException;
import java.nio.file.Path;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera; 


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  

  Joystick stick;
  Joystick stick2 = new Joystick(1);

  double swerveSpeed;
  double rotateSpeed;

  double armMotorSpeedUp = 1;
  double armMotorSpeedDown = 1;




  public CANSparkMax arm_motor = new CANSparkMax(18, MotorType.kBrushless);
  public CANSparkMax arm_motor2 = new CANSparkMax(19, MotorType.kBrushless);

  // Pneumatics
  public static final DoubleSolenoid telescopic_arm  = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 2);
 

    // Change this to match the name of your camera

    //PhotonCamera camera = new PhotonCamera("OV5647");

  //UsbCamera cam1;
  //CameraServer cam2;


 
 
  @Override
  public void robotInit() {

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    stick = m_robotContainer.m_driverController;
    swerveSpeed = Constants.DriveConstants.kMaxSpeedMetersPerSecond;
    rotateSpeed = Constants.DriveConstants.kMaxAngularSpeed;
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
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();



    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic () {
  
  
  
  m_autonomousCommand.execute();
 

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
    
    //cam1 = CameraServer.startAutomaticCapture(0);

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    if(stick.getRawButton(1)){
      Constants.DriveConstants.kMaxSpeedMetersPerSecond = 5;

    }
    else{
      Constants.DriveConstants.kMaxSpeedMetersPerSecond = 3;

    }

      
    
    //double armSpeedUp = stick2.getRawAxis(1) * armMotorSpeedUp;
    // double armSpeedDown = stick2.getRawAxis(1) * armMotorSpeedDown;
    
    
    //arm movement

    
    if(stick2.getRawButton(1 )){
      arm_motor.set(1);
      arm_motor2.set(1);
    }
    else if(stick2.getRawButton(4)){
      arm_motor.set(-1);
      arm_motor2.set(-1);
    }
    else{
      arm_motor.set(0);
      arm_motor2.set(0);
    }
    
  
 

    if(stick2.getRawButton(2)){
      telescopic_arm.set(Value.kReverse);
      System.out.println("switched reverse");
    }
    if(stick2.getRawButton(3)){
      telescopic_arm.set(Value.kForward);
      System.out.println("switched forward");
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
