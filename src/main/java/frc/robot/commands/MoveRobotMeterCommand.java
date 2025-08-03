// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.Thread.State;
import java.time.LocalTime;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveRobotMeterCommand extends Command {

  public static double startPos;
  public double endPos;
  public double tolerance = 0.4;
  public TrapezoidProfile.State startState;
  public TrapezoidProfile.State endState;
  public double startTime;
  // Timer
  private final Timer timer = new Timer();
    // Trapezoid profile
    public static final TrapezoidProfile trapezoidProfile =
    new TrapezoidProfile(
        new TrapezoidProfile.Constraints(
            Constants.maxVelocity,
            Constants.maxAcceleration));
  
  /** Creates a new MoveRobotCommand. */
  public MoveRobotMeterCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startPos = RobotContainer.driveSubsystem.getLeftMotorEncoder();
    endPos = startPos + 40;
    RobotContainer.driveSubsystem.robotDrive(.5,0);
    startState = new TrapezoidProfile.State(startPos, 0);
    endState = new TrapezoidProfile.State(endPos, 0);
    timer.restart();
    timer.start();
    System.out.println("Start state: " + startPos + "   End state: " + endPos);
    //new TrapezoidProfile.State(RobotContainer.driveSubsystem.leftMotor.getEncoder().getPosition(), 0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var s = trapezoidProfile.calculate(timer.get(), startState, endState);
    double p = s.velocity * 0.58 / 40.0;
    RobotContainer.driveSubsystem.robotDrive(p,0);
    //System.out.println("Power, " + p + "   Velocity: " + s.velocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Drive ended: " + interrupted + "\n Current encoder: " + RobotContainer.driveSubsystem.getLeftMotorEncoder());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(RobotContainer.driveSubsystem.getLeftMotorEncoder() - endPos) < tolerance);
  }
}
