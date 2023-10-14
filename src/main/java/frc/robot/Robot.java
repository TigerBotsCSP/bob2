// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  String trajectoryJSON = "paths/Straight.wpilib.json";
  Trajectory trajectory = new Trajectory();
  
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
   } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
   }

    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  public PIDController m_pid = new PIDController(0.01, .1, 0);
  public double m_currentRotation = 0;

  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

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
    // Set the intaker as going in
    //m_robotContainer.m_arm.toggleIntaker();

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
    //if (!m_robotContainer.m_arm.m_positioning) {
    //  double value = m_pid.calculate(m_robotContainer.m_arm.m_rotation.getEncoder().getPosition(), m_currentRotation);
    //  m_robotContainer.m_arm.setRotation(value * .3);
    //}

    // Move the arm based on joystick values
    //m_robotContainer.m_arm.setRotation(m_robotContainer.m_armController.getLeftY());
    
    // Change intaker direction
    //if (m_robotContainer.m_armController.getAButtonPressed()) {
    //  m_robotContainer.m_arm.toggleIntaker();
    //}

    // One button shooting
    //if (m_robotContainer.m_armController.getBButtonPressed()) {
    //  m_robotContainer.m_arm.shootOut();
    //}

    // Positioning
    //if (m_robotContainer.m_armController.getYButtonPressed()) {
    //  m_robotContainer.m_arm.position();
    //}

    // Data
    //SmartDashboard.putNumber("Encoder Position",
    //    m_robotContainer.m_arm.m_rotation.getEncoder().getPosition());
    //SmartDashboard.putNumber("Encoder Velocity",
    //    m_robotContainer.m_arm.m_rotation.getAbsoluteEncoder(Type.kDutyCycle).getVelocity());

    //    m_currentRotation = m_robotContainer.m_arm.m_rotation.getEncoder().getPosition();

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
