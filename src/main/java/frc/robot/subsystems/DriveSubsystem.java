// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SignalsConfig;
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
  private static DifferentialDrive chassis;
  private static SparkMax leftMotor;
  private static SparkMax rightMotor;
  private static SparkMaxConfig configRight;
  private static SparkMaxConfig configLeft;

  private RelativeEncoder leftMotorEncoder; 
  private RelativeEncoder rightMotorEncoder;


  private SparkClosedLoopController leftPIDController;
  private SparkClosedLoopController rightPIDController;

  public DriveSubsystem() {
    leftMotor = new SparkMax(57, MotorType.kBrushless);
    rightMotor = new SparkMax(54, MotorType.kBrushless);

    leftMotorEncoder = leftMotor.getEncoder();
    rightMotorEncoder = rightMotor.getEncoder();

    configRight = new SparkMaxConfig();
    configLeft = new SparkMaxConfig();

    configurePID();
    configureMotors();

    chassis = new DifferentialDrive(rightMotor, leftMotor);
    chassis.setSafetyEnabled(false);
  }

  private void configurePID() {
    leftPIDController = leftMotor.getClosedLoopController();
    rightPIDController = rightMotor.getClosedLoopController();
    configLeft.voltageCompensation(Constants.nominalVoltage);
    configRight.voltageCompensation(Constants.nominalVoltage);

    configLeft.openLoopRampRate(Constants.rampRate);
    configLeft.closedLoopRampRate(Constants.rampRate);
    configRight.openLoopRampRate(Constants.rampRate);
    configRight.closedLoopRampRate(Constants.rampRate);

    SignalsConfig signalsConfigLeft = new SignalsConfig();
    SignalsConfig signalsConfigRight = new SignalsConfig();

     // kstatus1
    signalsConfigLeft.motorTemperaturePeriodMs(50);
    signalsConfigLeft.primaryEncoderVelocityPeriodMs(10);

    // kstatus2
    signalsConfigLeft.primaryEncoderPositionPeriodMs(10);

    // kstatus1
    signalsConfigRight.motorTemperaturePeriodMs(50);
    signalsConfigRight.primaryEncoderVelocityPeriodMs(10);

    // kstatus2
    signalsConfigRight.primaryEncoderPositionPeriodMs(10);

    configRight.apply(signalsConfigRight);
    configLeft.apply(signalsConfigLeft);

    leftMotor.setCANTimeout(0);
    rightMotor.setCANTimeout(0);




    ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();
    closedLoopConfig.p(PIDConstants.kP);
    closedLoopConfig.i(PIDConstants.kI);
    closedLoopConfig.d(PIDConstants.kD);
    closedLoopConfig.outputRange(-0.6, 0.6);
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
    chassis.arcadeDrive((moveX), moveY);
  }

  public void runMotors() {
    rightMotor.set(.18);
    leftMotor.set(.18);
    double leftEncoder = leftMotor.getEncoder().getPosition(); // Gets values to move at
    System.out.println("Encoder: " + leftEncoder);
  }

  public void stopMotors() {
    rightMotor.set(0);
    leftMotor.set(0);
  }

  public static SparkMax getLeftMotor() {
    return leftMotor;
  }

  public static SparkMax getRightMotor() {
    return rightMotor;
  }

  public double getLeftMotorEncoder() {
    return leftMotor.getEncoder().getPosition();
  }

  public double getRightMotorEncoder() {
    return rightMotor.getEncoder().getPosition();
  }

  public void SetLeftMotorPID(double pos) {

    System.out.print("LP:"+pos);
    leftPIDController.setReference(pos, ControlType.kPosition);
  }
  
  public void SetRightMotorPID(double pos) {
    rightPIDController.setReference(pos, ControlType.kPosition);
  }
  

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
