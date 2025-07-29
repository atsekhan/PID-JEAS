// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */
  public static DifferentialDrive chassis; 
  public static SparkMax leftMotor;
  public static SparkMax rightMotor;
  public static SparkMaxConfig configRight = new SparkMaxConfig();
  public static SparkMaxConfig configLeft = new SparkMaxConfig();


  public DriveSubsystem() {
    leftMotor = new SparkMax(57, MotorType.kBrushless);
    rightMotor = new SparkMax(54, MotorType.kBrushless);
    configRight.idleMode(IdleMode.kBrake);
    configLeft.idleMode(IdleMode.kBrake);
     leftMotor.configure(configLeft, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
     configRight.inverted(true);
     rightMotor.configure(configRight, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    chassis = new DifferentialDrive(rightMotor, leftMotor);
  }

  public void robotDrive(double moveX, double moveY){
    chassis.arcadeDrive(moveX, moveY);
  }

  // public static double getMotorVoltage() {
  //   return leftMotor.getAppliedOutput();
  // }

  public void runSingleMotor(){
    rightMotor.set(.3);
  }

  public void stopSingleMotor(){
    rightMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
