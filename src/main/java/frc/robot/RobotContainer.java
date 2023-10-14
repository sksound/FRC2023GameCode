// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;



import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {


  // The robot's subsystems
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // The driver's controller
  public Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
  public Joystick m_driverController2 = new Joystick(OIConstants.kDriverControllerPort2);

  // public static final HashMap <String, Command> autoEventMap = new HashMap<>();

  double armState;

  
  public boolean isOpen = false;
  
    

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband(-m_driverController.getRawAxis(1), 0.06),
                MathUtil.applyDeadband(-m_driverController.getRawAxis(0), 0.06),
                MathUtil.applyDeadband(-m_driverController.getRawAxis(4), 0.06),

                true),
            m_robotDrive)

            //make deadbands for 2nd controller
            
            
            
            );
  }

  public void setArmState(double newArmState){

    this.armState = newArmState;
    

  }

   


  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */


  

public class armSet extends CommandBase{

double newArmPosSet;
boolean isDone = false;

public armSet(double newArmPos){
      

newArmPosSet = newArmPos;
    
    
}
@Override
public void initialize(){
  setArmState(newArmPosSet);
  isDone = true;
}
@Override
public boolean isFinished() {
  return isDone;
}
    
    
}


public class clawOpen extends CommandBase{

  boolean isDone = false;

public clawOpen(){
        

  new WaitCommand(1.5);
      
}

@Override
public void initialize(){
  
  isOpen = true;
  isDone = true;
}

public boolean isFinished(){
  return isDone;
}
      
      
}

public class clawClose extends CommandBase{

  boolean isDone = false;

  public clawClose(){
          
    new WaitCommand(1.5);
  
        
  }

  @Override
public void initialize(){
  
  isOpen = false;
  isDone = true;
}

public boolean isFinished(){
  return isDone;
}
        
        
  }


//stops all motors
public class stopRobot extends CommandBase{

public stopRobot(DriveSubsystem drive){
  
drive.stopAllMotors();


}


}

//used to loop autbalance method
public class autoBalance extends CommandBase{

  DriveSubsystem driveSet;

  public autoBalance(DriveSubsystem drive){
    driveSet = drive;
    //drive.autoBalance();

  }
  @Override
  public void initialize(){
    driveSet.dummyBoo = true;
  }


}








//picks up cube, places, and autobalances
public Command newCommand(){

  var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);


  //autoEventMap.put("event", new PrintCommand("event passed"));
  //autoEventMap.put("autoBalance", new InstantCommand(m_robotDrive::autoBalance, m_robotDrive));



  List<PathPlannerTrajectory> autoPaths = PathPlanner.loadPathGroup("superCoolAuto", 2, 3);

  Command autoTest = new SequentialCommandGroup(
    //new FollowPathWithEvents(new SwerveControllerCommand(autoPaths.get(0), m_robotDrive::getPose, DriveConstants.kDriveKinematics, new PIDController(AutoConstants.kPXController, 0, 0), new PIDController(AutoConstants.kPYController, 0, 0), thetaController, m_robotDrive::setModuleStates, m_robotDrive), autoPaths.get(0).getMarkers(), autoEventMap),
    new armSet(.5),
    new InstantCommand(m_robotDrive::stopAllMotors, m_robotDrive),
    new WaitCommand(2.5),
    new InstantCommand(m_robotDrive::stopAllMotors, m_robotDrive),
    //new FollowPathWithEvents(new SwerveControllerCommand(
    //autoPaths.get(1), m_robotDrive::getPose, DriveConstants.kDriveKinematics, new PIDController(AutoConstants.kPXController, 0, 0), new PIDController(AutoConstants.kPYController, 0, 0), thetaController, m_robotDrive::setModuleStates, m_robotDrive), autoPaths.get(1).getMarkers(), autoEventMap),
    new InstantCommand(m_robotDrive::stopAllMotors, m_robotDrive),
    new WaitCommand(2.5),
    new InstantCommand(m_robotDrive::stopAllMotors, m_robotDrive),
    //new FollowPathWithEvents(new SwerveControllerCommand(
    //autoPaths.get(2), m_robotDrive::getPose, DriveConstants.kDriveKinematics, new PIDController(AutoConstants.kPXController, 0, 0), new PIDController(AutoConstants.kPYController, 0, 0), thetaController, m_robotDrive::setModuleStates, m_robotDrive), autoPaths.get(2).getMarkers(), autoEventMap),
    new InstantCommand(m_robotDrive::stopAllMotors, m_robotDrive),
    new autoBalance(m_robotDrive),
    new WaitCommand(10)
  );

  
  m_robotDrive.resetOdometry(autoPaths.get(0).getInitialPose());
  //m_robotDrive.resetOdometry(autoPaths.get(1).getInitialPose());
 
  return autoTest.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  
  
}
//autobalance testing
public Command autoBalance(){

  var thetaController = new ProfiledPIDController(
    AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
thetaController.enableContinuousInput(-Math.PI, Math.PI);


//autoEventMap.put("autoBalance", new InstantCommand(m_robotDrive::autoBalance, m_robotDrive));



List<PathPlannerTrajectory> autoPaths = PathPlanner.loadPathGroup("testPath", 2, 3);

Command autoBalanceTest = new SequentialCommandGroup(
//new FollowPathWithEvents(new SwerveControllerCommand(
//autoPaths.get(0), m_robotDrive::getPose, DriveConstants.kDriveKinematics, new PIDController(AutoConstants.kPXController, 0, 0), new PIDController(AutoConstants.kPYController, 0, 0), thetaController, m_robotDrive::setModuleStates, m_robotDrive), autoPaths.get(0).getMarkers(), autoEventMap),
new InstantCommand(m_robotDrive::stopAllMotors, m_robotDrive),
new autoBalance(m_robotDrive),
new WaitCommand(5)











);


m_robotDrive.resetOdometry(autoPaths.get(0).getInitialPose());
//m_robotDrive.resetOdometry(autoPaths.get(1).getInitialPose());

return autoBalanceTest.andThen(() -> m_robotDrive.drive(0, 0, 0, false));


}

public Command armStuff(){

  var thetaController = new ProfiledPIDController(
    AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
thetaController.enableContinuousInput(-Math.PI, Math.PI);


//autoEventMap.put("autoBalance", new InstantCommand(m_robotDrive::autoBalance, m_robotDrive));



List<PathPlannerTrajectory> autoPaths = PathPlanner.loadPathGroup("testPath", 2, 3);

Command armTest = new SequentialCommandGroup(
new clawClose(),
new armSet(.5),
new clawOpen()

);


m_robotDrive.resetOdometry(autoPaths.get(0).getInitialPose());
//m_robotDrive.resetOdometry(autoPaths.get(1).getInitialPose());

return armTest.andThen(() -> m_robotDrive.drive(0, 0, 0, false));


}

public Command simpleCubeAuto(){
  var thetaController = new ProfiledPIDController(
    AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
thetaController.enableContinuousInput(-Math.PI, Math.PI);


//autoEventMap.put("autoBalance", new InstantCommand(m_robotDrive::autoBalance, m_robotDrive));



List<PathPlannerTrajectory> autoPaths = PathPlanner.loadPathGroup("simpleCubePoint", 2, 3);

 m_robotDrive.resetOdometry(autoPaths.get(0).getInitialHolonomicPose());

Command simpleCube = new SequentialCommandGroup(
  //new FollowPathWithEvents(new SwerveControllerCommand(autoPaths.get(0), m_robotDrive::getPose, DriveConstants.kDriveKinematics, new PIDController(AutoConstants.kPXController, 0, 0), new PIDController(AutoConstants.kPYController, 0, 0), thetaController, m_robotDrive::setModuleStates, m_robotDrive), autoPaths.get(0).getMarkers(), autoEventMap),
  new InstantCommand(m_robotDrive::stopAllMotors, m_robotDrive),
  new WaitCommand(5),
  new InstantCommand(m_robotDrive::stopAllMotors, m_robotDrive)
  //new FollowPathWithEvents(new SwerveControllerCommand(autoPaths.get(1), m_robotDrive::getPose, DriveConstants.kDriveKinematics, new PIDController(AutoConstants.kPXController, 0, 0), new PIDController(AutoConstants.kPYController, 0, 0), thetaController, m_robotDrive::setModuleStates, m_robotDrive), autoPaths.get(1).getMarkers(), autoEventMap)
);


m_robotDrive.resetOdometry(autoPaths.get(0).getInitialPose());
//m_robotDrive.resetOdometry(autoPaths.get(1).getInitialPose());

return simpleCube.andThen(() -> m_robotDrive.drive(0, 0, 0, false));


}

public Command followTrajectoryCommand() {

  HashMap <String, Command> autoEventMap = new HashMap<>();

  autoEventMap.put("armDown", new armSet(.24));
  autoEventMap.put("armUp", new armSet(.5));
  autoEventMap.put("setConePos", new armSet(.357));
  autoEventMap.put("pickUpCone", new clawClose());
  autoEventMap.put("dropCone", new clawOpen());
  autoEventMap.put("autoBalance", new autoBalance(m_robotDrive));
  autoEventMap.put("initArmDown", new armSet(.357));
  autoEventMap.put("dropCube", new clawOpen());
  autoEventMap.put("initArmBackUp", new armSet(.5));
  autoEventMap.put("wait1", new WaitCommand(2));
  autoEventMap.put("coneArmUp", new armSet(.5));

  List<PathPlannerTrajectory> autoPaths = PathPlanner.loadPathGroup("ConeCubeAutoBalance", 3, 3);


   SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    m_robotDrive::getPose, // Pose2d supplier
    m_robotDrive::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
    DriveConstants.kDriveKinematics, // SwerveDriveKinematics
    new PIDConstants(1, 0.0, .25), // PID constants to correct for translation error (used to create the X and Y PID controllers)
    new PIDConstants(1, 0.0, 0.25), // PID constants to correct for rotation error (used to create the rotation controller)
    m_robotDrive::setModuleStates, // Module states consumer used to output to the drive subsystem
    autoEventMap,
    true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    m_robotDrive // The drive subsystem. Used to properly set the requirements of path following commands
);

Command fullAuto = autoBuilder.fullAuto(autoPaths);

return fullAuto;
  

}

public Command coneCubeCorner(){
  HashMap <String, Command> autoEventMap = new HashMap<>();

  

  autoEventMap.put("initClose", new clawClose());
  autoEventMap.put("armDown", new armSet(.765));
  autoEventMap.put("armUp", new armSet(.5));
  autoEventMap.put("setConePos", new armSet(.357));
  autoEventMap.put("pickUpCone", new clawClose());
  autoEventMap.put("dropCone", new clawOpen());
  autoEventMap.put("autoBalance", new autoBalance(m_robotDrive));
  autoEventMap.put("initArmDown", new armSet(.357));
  autoEventMap.put("dropCube", new clawOpen());
  autoEventMap.put("initArmBackUp", new armSet(.5));
  autoEventMap.put("wait1", new WaitCommand(2));
  autoEventMap.put("coneArmUp", new armSet(.5));

  List<PathPlannerTrajectory> autoPaths = PathPlanner.loadPathGroup("ConeCubeAutoBalanceBackwards", 3, 3);


   SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    m_robotDrive::getPose, // Pose2d supplier
    m_robotDrive::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
    DriveConstants.kDriveKinematics, // SwerveDriveKinematics
    new PIDConstants(4, 0.0, .25), // PID constants to correct for translation error (used to create the X and Y PID controllers)
    new PIDConstants(4, 0.0, 0.25), // PID constants to correct for rotation error (used to create the rotation controller)
    m_robotDrive::setModuleStates, // Module states consumer used to output to the drive subsystem
    autoEventMap,
    true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    m_robotDrive // The drive subsystem. Used to properly set the requirements of path following commands
);

Command fullAuto = autoBuilder.fullAuto(autoPaths);

return fullAuto;
  

}

public Command cubeAutoBalance(){

  System.out.println("code for autobalance worked");

  HashMap <String, Command> autoEventMap = new HashMap<>();

  autoEventMap.put("initArmDown", new armSet(.357));
  autoEventMap.put("dropCube", new clawOpen());
  autoEventMap.put("initClose", new clawClose());
  autoEventMap.put("armUp", new armSet(.5));
  autoEventMap.put("autoBalance", new autoBalance(m_robotDrive));
  autoEventMap.put("initArmBackUp", new armSet(.5));
  autoEventMap.put("coneArmUp", new armSet(.5));

  List<PathPlannerTrajectory> autoPaths = PathPlanner.loadPathGroup("cubeAutoBalance", 3, 3);


   SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    m_robotDrive::getPose, // Pose2d supplier
    m_robotDrive::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
    DriveConstants.kDriveKinematics, // SwerveDriveKinematics
    new PIDConstants(4, 0.0, .25), // PID constants to correct for translation error (used to create the X and Y PID controllers)
    new PIDConstants(4, 0.0, 0.25), // PID constants to correct for rotation error (used to create the rotation controller)
    m_robotDrive::setModuleStates, // Module states consumer used to output to the drive subsystem
    autoEventMap,
    true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    m_robotDrive // The drive subsystem. Used to properly set the requirements of path following commands
);

Command fullAuto = autoBuilder.fullAuto(autoPaths);

return fullAuto;
  

}


public Command getAutonomousCommand() {

 
    PathPlannerTrajectory examplePath = PathPlanner.loadPath("autoTest", new PathConstraints(4, 3));

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        examplePath,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

    

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);


    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(examplePath.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }




 
  
}

