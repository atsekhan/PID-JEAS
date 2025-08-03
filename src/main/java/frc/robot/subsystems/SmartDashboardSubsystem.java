// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.DriveManuallyCommand;

public class SmartDashboardSubsystem extends SubsystemBase {
  /** Creates a new SmartDashboardSubsystem. */
  public SmartDashboardSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("Motor Power: ", DriveSubsystem.getMotorVoltage());
    // SmartDashboard.putNumber("Y axis: ", DriveManuallyCommand.move);
    // SmartDashboard.putNumber("X axis: ", DriveManuallyCommand.turn);
    SmartDashboard.putNumber("Left Encoder: ", DriveSubsystem.getLeftMotor().getEncoder().getPosition());
    SmartDashboard.putNumber("Right Encoder: ", DriveSubsystem.getRightMotor().getEncoder().getPosition());

  }
}
