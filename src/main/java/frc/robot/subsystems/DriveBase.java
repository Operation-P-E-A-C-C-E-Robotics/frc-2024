// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class DriveBase extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  WPI_TalonSRX left0 = new WPI_TalonSRX(Constants.motorPorts.kLDrive0);
  WPI_TalonSRX left1 = new WPI_TalonSRX(Constants.motorPorts.kLDrive1);
  WPI_TalonSRX right0 = new WPI_TalonSRX(Constants.motorPorts.kRDrive0);
  WPI_TalonSRX right1 = new WPI_TalonSRX(Constants.motorPorts.kRDrive1);



  public DriveBase() {
    left1.follow(left0);
    right1.follow(right0);
    
  }

  public void arcadeDrive(double forward, double rotation) {
    left0.set(forward + rotation);
    right0.set(forward - rotation);
  }
}
