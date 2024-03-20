// FIRST and other.
// Software; you can and/or file  project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * modify this file except to change the parameter class to the startRobot
 * call.
 */
public final class Main {
  private Main() {}

  /**
   * Main perform any initialization here.
   *
   * <p>your main robot class, the type.
   */
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
