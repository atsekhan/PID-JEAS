// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PIDConstants;
import frc.robot.RobotContainer;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */
  public static DifferentialDrive chassis;
  public static SparkMax leftMotor;
  public static SparkMax rightMotor;
  public static SparkMaxConfig configRight = new SparkMaxConfig();
  public static SparkMaxConfig configLeft = new SparkMaxConfig();

  private SparkClosedLoopController leftPIDController;
  private SparkClosedLoopController rightPIDController;

  public DriveSubsystem() {
    leftMotor = new SparkMax(57, MotorType.kBrushless);
    rightMotor = new SparkMax(54, MotorType.kBrushless);

    configurePID();
    configureMotors();

    chassis = new DifferentialDrive(rightMotor, leftMotor);
    chassis.setSafetyEnabled(false);
  }

  private void configurePID() {
    leftPIDController = leftMotor.getClosedLoopController();
    rightPIDController = rightMotor.getClosedLoopController();
    // configRight.voltageCompensation(Constants.nominalVoltage);
    configLeft.voltageCompensation(Constants.nominalVoltage);
    configRight.voltageCompensation(Constants.nominalVoltage);

    configLeft.openLoopRampRate(Constants.rampRate);
    configLeft.closedLoopRampRate(Constants.rampRate);
    configRight.openLoopRampRate(Constants.rampRate);
    configRight.closedLoopRampRate(Constants.rampRate);

    ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();
    closedLoopConfig.p(PIDConstants.kP);
    closedLoopConfig.i(PIDConstants.kI);
    closedLoopConfig.d(PIDConstants.kD);
    closedLoopConfig.outputRange(-1, 1);
    configLeft.apply(closedLoopConfig);
    configRight.apply(closedLoopConfig);
  }

  private void configureMotors() {
    configRight.idleMode(IdleMode.kBrake);
    configLeft.idleMode(IdleMode.kBrake);
    leftMotor.configure(configLeft, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    configRight.inverted(true);
    rightMotor.configure(configRight, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
  }

  public void robotDrive(double moveX, double moveY) {
    chassis.arcadeDrive(moveX, moveY);
  }

  public void runMotors() {
    rightMotor.set(.58);
    leftMotor.set(.58);
    double leftEncoder = leftMotor.getEncoder().getPosition(); // Gets values to move at
    System.out.println("Encoder: " + leftEncoder);
  }

  public void stopMotors() {
    rightMotor.set(0);
    leftMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
