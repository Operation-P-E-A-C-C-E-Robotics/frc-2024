// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.telemetry.ControlSystemTelemetry;
import frc.lib.telemetry.MultiTracers;
import frc.robot.auto.AutoTakeTwo;
import frc.robot.subsystems.Swerve;

public class Robot extends TimedRobot {
  private Timer scheduleTimer = new Timer(); //used to log loop time

  public Robot() {
    super(Constants.period); // use a custom loop time
    RobotContainer.getInstance(); // make sure RobotContainer is initialized
  }

  @Override
  public void robotInit() {
    MultiTracers.disable(); //change to enable to print timing debugging info to the console
    //log data from network tables (SmartDashboard, etc.)
    DataLogManager.start();
    //only log network tables data when the robot is enabled, to keep the logs from taking forever to open
    DataLogManager.logNetworkTables(false); 
    DriverStation.startDataLog(DataLogManager.getLog());

    System.out.println("Robot Initialized");
    System.out.println("yay the software didn't crash yet");
  }

  @Override
  public void robotPeriodic() {
    //run the robot
    scheduleTimer.reset();
    scheduleTimer.start();

    RobotContainer.getInstance().run(); // This does all the important stuff

    //log loop time and other RIO data
    ControlSystemTelemetry.update(null, scheduleTimer.get());
  }

  @Override
  public void disabledInit() {
    DataLogManager.logNetworkTables(false); //stop logging network tables data when the robot is disabled
    System.out.println("Robot Disabled");
    System.out.println("let's hope it doesn't move lol");
  }

  @Override
  public void disabledPeriodic() {
    RobotContainer.getInstance().zeroAutoHeading();
    Swerve.getInstance().periodic();
    AutoTakeTwo.fourNote.reset();
  }

  @Override
  public void autonomousInit() {
    RobotContainer.getInstance().resetAuto();
    DataLogManager.logNetworkTables(true); //start logging network tables data when the robot is enabled
    System.out.println("Robot Autonomous");
    System.out.println("EVERYBODY PANIC PEACCY IS RUNNING AUTONOMOUS AND HE DOESN'T KNOW WHAT HE'S DOING");
  }

  @Override
  public void teleopInit() {
    DataLogManager.logNetworkTables(true); //start logging network tables data when the robot is enabled
    System.out.println("Robot Teleop");
    System.out.println("EVERYBODY RUN PEACCY IS DRIVING THE ROBOT AND HE WILL CRASH IT VERY SOON");
  }

  @Override
  public void testInit() {
    DataLogManager.logNetworkTables(true);
    System.out.println("Robot Test");
    System.out.println("ok but what are yall doing i don't even know why you'd be running in test mode");
  }
}
