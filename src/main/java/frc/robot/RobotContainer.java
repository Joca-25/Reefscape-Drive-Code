// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Commands.ElevatorDown;
import frc.robot.Commands.ElevatorUp;
import frc.robot.Commands.PivotShooterDown;
import frc.robot.Commands.PivotShooterUp;
import frc.robot.Commands.ShooterInward;
import frc.robot.Commands.ShooterOutward;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.TestConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

import com.revrobotics.spark.SparkMax;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ShooterSubsystem shooter;
  private final ElevatorSubsystem elevator; 
  
  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_controllerTwo = new XboxController(1);
 /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    //initialize subsystems
    shooter = new ShooterSubsystem(20,21);
    elevator = new ElevatorSubsystem(22);
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
            
    shooter.setDefaultCommand( 
        new RunCommand(
          () -> {
            shooter.pivotStop();
            shooter.shooterStop();},
          shooter));

    elevator.setDefaultCommand( new RunCommand(
      () -> elevator.stop(),
      elevator));
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
    // new JoystickButton(m_driverController, Button.kLeftBumper.value)
    //     .whileTrue(new RunCommand(
    //         () -> m_robotDrive.setX()
    // ));
    
    JoystickButton aButton = new JoystickButton (m_controllerTwo, Button.kA.value); // For the "A" button
    aButton.whileTrue(new ShooterOutward(shooter,3));
    
    JoystickButton bButton = new JoystickButton (m_controllerTwo, Button.kB.value); // For the "B" button
    bButton.whileTrue(new ShooterInward(shooter,3));

    JoystickButton yButton = new JoystickButton (m_controllerTwo, Button.kY.value); // For the "Y" button
    yButton.whileTrue(new PivotShooterDown(shooter,3));
    
    JoystickButton xButton = new JoystickButton (m_controllerTwo, Button.kX.value); // For the "X" button
    xButton.whileTrue(new PivotShooterUp(shooter,3));

    JoystickButton LbBumperButton = new JoystickButton (m_controllerTwo, Button.kLeftBumper.value); // For the "X" button
     LbBumperButton.whileTrue(new ElevatorDown(elevator,(int)elevator.getDistance()+10));
     
    JoystickButton RbBumperButton = new JoystickButton (m_controllerTwo, Button.kRightBumper.value); // For the "X" button
     RbBumperButton.whileTrue(new ElevatorUp(elevator,3));
    
     
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
public Command SwerveControllerCommand(){
  TrajectoryConfig config = new TrajectoryConfig(
    AutoConstants.kMaxSpeedMetersPerSecond,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(DriveConstants.kDriveKinematics);

  // An example trajectory to follow. All units in meters.
  Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(0, 0, new Rotation2d(0)),
      // Go forward 
      List.of(),
    //List.of (new Translation2d(0, .33),new Translation2d(0, .66)),
      // This new code should make the robot go forward 1 meter.
      new Pose2d(1, 0, new Rotation2d(0)),
      config);


  var thetaController = new ProfiledPIDController(
      AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
  // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return SwerveControllerCommand().andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  
}


   public Command autoLeftSide(){
        // Create config for trajectory
     TrajectoryConfig config = new TrajectoryConfig(
      AutoConstants.kMaxSpeedMetersPerSecond,
      AutoConstants.kMaxAccelerationMetersPerSecondSquared)
      // Add kinematics to ensure max speed is actually obeyed
      .setKinematics(DriveConstants.kDriveKinematics);

  // An example trajectory to follow. All units in meters.
  Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(0, 0, new Rotation2d(0)),
      // Go forward 
      List.of(),
     //List.of (new Translation2d(0, .33),new Translation2d(0, .66)),
      // This new code should make the robot go forward 1 meter.
      new Pose2d(1, 0, new Rotation2d(0)),
      config);


  var thetaController = new ProfiledPIDController(
      AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
 // thetaController.enableContinuousInput(-Math.PI, Math.PI);

  SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
      exampleTrajectory,
      m_robotDrive::getPose, // Functional interface to feed supplier
      DriveConstants.kDriveKinematics,

      // Position controllers
      new PIDController(AutoConstants.kPXController, 0, 0),
      new PIDController(AutoConstants.kPYController, 0, 0),
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

  // Reset odometry to the starting pose of the trajectory.
  m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

  // Run path following command, then stop at the end.
  Command temp= swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    return Commands.sequence(Commands.parallel(temp, new ElevatorUp(null, 0)), 
                        new PivotShooterUp(null,3));


   }

   public Command autoRightSide(){
       // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
      AutoConstants.kMaxSpeedMetersPerSecond,
      AutoConstants.kMaxAccelerationMetersPerSecondSquared)
      // Add kinematics to ensure max speed is actually obeyed
      .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
      new Pose2d(0, 0, new Rotation2d(0)),
      // Go forward 
      List.of(),
     //List.of (new Translation2d(0, .33),new Translation2d(0, .66)),
      // This new code should make the robot go forward 1 meter.
      new Pose2d(2, 0, new Rotation2d(60)),
      config);


    var thetaController = new ProfiledPIDController(
      AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
      exampleTrajectory,
      m_robotDrive::getPose, // Functional interface to feed supplier
      DriveConstants.kDriveKinematics,

      // Position controllers
      new PIDController(AutoConstants.kPXController, 0, 0),
      new PIDController(AutoConstants.kPYController, 0, 0),
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    Command temp= swerveControllerCommand.andThen(() -> m_robotDrive.drive(2, 0, 60, false));
    return Commands.sequence(Commands.parallel(temp, new ElevatorUp(null, 0)), 
                        new PivotShooterUp(null,3), new PivotShooterDown(null,3));
    
   }

   public Command autoMiddle(){
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
      AutoConstants.kMaxSpeedMetersPerSecond,
      AutoConstants.kMaxAccelerationMetersPerSecondSquared)
      // Add kinematics to ensure max speed is actually obeyed
      .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(0, 0, new Rotation2d(0)),
      // Go forward 
      List.of(),
     //List.of (new Translation2d(0, .33),new Translation2d(0, .66)),
      // This new code should make the robot go forward 1 meter.
      new Pose2d(1, 0, new Rotation2d(0)),
      config);


    var thetaController = new ProfiledPIDController(
          AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
      exampleTrajectory,
      m_robotDrive::getPose, // Functional interface to feed supplier
      DriveConstants.kDriveKinematics,

      // Position controllers
      new PIDController(AutoConstants.kPXController, 0, 0),
      new PIDController(AutoConstants.kPYController, 0, 0),
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    Command temp= swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    return Commands.sequence(Commands.parallel(temp, new ElevatorUp(null, 0)), 
                        new PivotShooterUp(null,3));
    // TrajectoryConfig config = new TrajectoryConfig(
    //   AutoConstants.kMaxSpeedMetersPerSecond,
    //   AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //   // Add kinematics to ensure max speed is actually obeyed
    //   .setKinematics(DriveConstants.kDriveKinematics);

    // Trajectory middleTrajectory = TrajectoryGenerator.generateTrajectory(
    //   // Start at the origin facing the +X direction
    //   new Pose2d(0, 0, new Rotation2d(0)),
    //   // Go forward 
    //   List.of(),
    //  //List.of (new Translation2d(0, .33),new Translation2d(0, .66)),
    //   // This new code should make the robot go forward 1 meter.
    //   new Pose2d(1, 0, new Rotation2d(0)),
    //   config);


   }


  public Command getAutonomous() {
   
     
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Go forward 
        List.of(),
       //List.of (new Translation2d(0, .33),new Translation2d(0, .66)),
        // This new code should make the robot go forward 1 meter.
        new Pose2d(1, 0, new Rotation2d(0)),
        config); 


    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
   // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }

  public Command t_encoderLimit(){
    return null;
  }

  public Command t_encoderCount(){
    return null; 
  }

  //Commands for testing drivetrain
  public Command t_driveDirection(){
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        TestConstants.kMaxSpeedMetersPerSecond,
        TestConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Check if these values are relative or absolute
        List.of(new Translation2d(0, 1), new Translation2d(-1, 0), new Translation2d(0, -1),
        new Translation2d(1, 0)),
        
        // End where we started
        new Pose2d(0, 0, new Rotation2d(0)),
        config);
     var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  
  }

  public Command t_EncoderDistanceTracking(){
    return null;
  }

  public Command t_MechanismDirections(){
    return null;
  }

  public Command t_DrivetrainStartingPosition(){
  // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);
      return null;
  }

  //public Command t_PivotAngles(){}
    
  //Returns runcommand, returns the command that tells it to run at a certain speed
  public void Forward(SparkMax motor, int speed){
    // RunCommand rc = new RunCommand(
    //         () -> motor.set(speed),0
    //        );

    
  }
  public void Backward(SparkMax motor, int speed){
    // return RunCommand(
//              () -> motor.set(-speed,9
// );
  }
}
  //if commands don't work we can use periodic