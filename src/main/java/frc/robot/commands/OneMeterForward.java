// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.PIDConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OneMeterForward extends Command {
  private double startPos;
  private double endPos;
  private double tolerance = 0.4;
  //private TrapezoidProfile trapezoidProfile = new TrapezoidProfile(TrapezoidProfile.maxVelovity, TrapezoidProfile.maxAcceleration);

  
  PIDController pid = new PIDController(PIDConstants.kP, PIDConstants.kI, PIDConstants.kD);
  /** Creates a new OneMeterForward. */
  public OneMeterForward() {
    addRequirements(RobotContainer.driveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("starting forward");
    startPos = RobotContainer.driveSubsystem.leftMotor.getEncoder().getPosition(); // Gets values to move at
    endPos = startPos + 20; // 20 encoder ticks is 1 meter
    System.out.println("StarPos: " + startPos + "endPos: " + endPos);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //RobotContainer.driveSubsystem.robotDrive(pid.calculate(RobotContainer.driveSubsystem.leftMotor.getEncoder().getPosition(), endPos),0);
    
    double encoder = RobotContainer.driveSubsystem.leftMotor.getEncoder().getPosition();
    double p = pid.calculate(RobotContainer.driveSubsystem.leftMotor.getEncoder().getPosition(), endPos);
    //System.out.println("Encoder value: " + encoder + " Power: " + p + " End pos " + endPos);
    RobotContainer.driveSubsystem.robotDrive(p,0);
  }

  // Called once the command ends or is interrupted.

  @Override
  public void end(boolean interrupted) {
    System.out.println("inerrupted " + interrupted); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //System.out.println("FINISHED WOOO");
    return (Math.abs(RobotContainer.driveSubsystem.leftMotor.getEncoder().getPosition() - endPos) < tolerance);
  }
}
