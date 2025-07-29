// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveManuallyCommand extends Command {

  public DoubleSupplier turn;
  public DoubleSupplier move;

  /** Creates a new RunMotorCommand. */
  public DriveManuallyCommand(DoubleSupplier turn, DoubleSupplier move) {
    addRequirements(RobotContainer.driveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
    this.turn = turn;
    this.move = move;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.driveSubsystem.robotDrive(move.getAsDouble(), turn.getAsDouble());
    //System.out.println("Move: " + turn.getAsDouble() + "|| Turn: " + move.getAsDouble());
    // System.out.println("LeftEncoder: " + DriveSubsystem.leftMotor.getEncoder().getPosition()
    //                    + "RightEncoder: " + DriveSubsystem.rightMotor.getEncoder().getPosition());
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
