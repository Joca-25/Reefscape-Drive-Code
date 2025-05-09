// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
//import frc.robot.Constants.TestConstants.TEST_PLAN;
//Import encoder, find out which one we use
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  //private Command testCommand;
  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  ShuffleboardTab mainTab; 
  private Thread m_visionThread;

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    mainTab = Shuffleboard.getTab("main");
    m_robotContainer = new RobotContainer(mainTab);

    m_visionThread =
        new Thread(
            () -> {
              // Get the UsbCamera from CameraServer
              UsbCamera camera = CameraServer.startAutomaticCapture();
              // Set the resolution
              camera.setResolution(640, 480);

              // Get a CvSink. This will capture Mats from the camera
              CvSink cvSink = CameraServer.getVideo();
              // Setup a CvSource. This will send images back to the Dashboard
              CvSource outputStream = CameraServer.putVideo("Camera Stream", 640, 480);

              // Mats are very memory expensive. Lets reuse this Mat.
              Mat mat = new Mat();
              AprilTagFinder aprils = new AprilTagFinder();
              // This cannot be 'true'. The program will never exit if it is. This
              // lets the robot stop this thread when restarting robot code or
              // deploying.
              while (!Thread.interrupted()) {
                // Tell the CvSink to grab a frame from the camera and put it
                // in the source mat.  If there is an error notify the output.
                if (cvSink.grabFrame(mat) == 0) {
                  // Send the output the error.
                  outputStream.notifyError(cvSink.getError());
                  // skip the rest of the current iteration
                  continue;
                }
                int id = (int) SmartDashboard.getNumber("TagId", 0);
                double x = aprils.process(mat,id);
                SmartDashboard.putNumber("Camera Centers", x);
                // Put a rectangle on the image
                Imgproc.rectangle(
                    mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);
                // Give the output stream a new image to display
                outputStream.putFrame(mat);
              }
            });
    m_visionThread.setDaemon(true);
    m_visionThread.start(); 
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
    //m_autonomousCommand = m_robotContainer.getAutonomous();
    String autoSelected = SmartDashboard.getString("Auto","autoMiddle");
   
    switch (autoSelected){
      case "autoLeftSide":
        m_autonomousCommand= m_robotContainer.autoLeftSide();
        break;
      case "autoRightSide":
        m_autonomousCommand = m_robotContainer.autoRightSide();
        break;
      case "autoMiddle":
        m_autonomousCommand = m_robotContainer.autoMiddle();
        break;
      case "getAutonomous":
      m_autonomousCommand = m_robotContainer.getAutonomous();
      break;
    }
      // String autoSelected = SmartDashboard.getString("Auto Selector",
      // "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
      // = new MyAutoCommand(); break; case "Default Auto": default:
      // autonomousCommand = new ExampleCommand(); break; }
     

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {


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
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Elevator Distance", m_robotContainer.elevator.getDistance());
    double kP_ele = SmartDashboard.getNumber("kP_ele", 1);
    double kI_ele = SmartDashboard.getNumber("kI_ele", 0.5);
    double kD_ele = SmartDashboard.getNumber("kD_ele", 0);
    m_robotContainer.elevator.setPID(kP_ele,kI_ele,kD_ele); 
    SmartDashboard.putNumber("Shooter Distance", m_robotContainer.shooter.getShooterPosition());
    double kP_shoot = SmartDashboard.getNumber("kP_shoot", 1);
    double kI_shoot = SmartDashboard.getNumber("kI_shoot", 0.5);
    double kD_shoot = SmartDashboard.getNumber("kD_shoot", 0);
    m_robotContainer.elevator.setPID(kP_shoot,kI_shoot,kD_shoot);
    SmartDashboard.putNumber("Pivot Angle", m_robotContainer.shooter.getPivotPosition());
    double kP_piv = SmartDashboard.getNumber("kP_piv", 1);
    double kI_piv = SmartDashboard.getNumber("kI_piv", 0.5);
    double kD_piv = SmartDashboard.getNumber("kD_piv", 0);
    m_robotContainer.elevator.setPID(kP_piv,kI_piv,kD_piv);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    //Get TEST chosen on smartdashboard
    //Example on line 63
 
   /*  switch (TEST){
      case ENCODERDIST_TRACKING:
        testCommand = m_robotContainer.t_EncoderDistanceTracking();
        break;
      case MECHANISM_DIRECTION:
        testCommand = m_robotContainer.t_MechanismDirections();
        break;
      case  DRIVETRAIN_STARTINGPOSITION:
        testCommand = m_robotContainer.t_DrivetrainStartingPosition();
        break;
      case PIVOT_ANGLES:
        testCommand = m_robotContainer.t_PivotAngles();
        break;
      default:
        testCommand = null;
        break;
    } */

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    /*switch(TEST){
      case ENCODER_COUNT:
        //Call encoder function here
       // int encoder = encoder.get(); 
        break;
      default:
        //put placeholder code
        break;
    }*/
  }
}
