// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.PIDConstants;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HardwarePIDMeterCommand extends Command {  
  private SparkClosedLoopController leftPIDController;
  private SparkClosedLoopController rightPIDController;
  public double leftStartPos;
  public double leftEndPos;
  public double rightStartPos;
  public double rightEndPos;
  public double tolerance = .4;

  /** Creates a new HardwarePIDMeterCommand. */
  public HardwarePIDMeterCommand() {
    addRequirements(RobotContainer.driveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leftStartPos = RobotContainer.driveSubsystem.leftMotor.getEncoder().getPosition(); // Gets values to move at
    leftEndPos = leftStartPos + 20; // 20 encoder ticks is 1 meter

    rightStartPos = RobotContainer.driveSubsystem.rightMotor.getEncoder().getPosition(); // Gets values to move at
    rightEndPos = rightStartPos + 20; // 20 encoder ticks is 1 meter

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Drive ended: " + interrupted + "\n Current encoder: " + RobotContainer.driveSubsystem.leftMotor.getEncoder().getPosition());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(RobotContainer.driveSubsystem.leftMotor.getEncoder().getPosition() - leftEndPos) < tolerance);
  }
}
