// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UpperWrist extends SubsystemBase {

  private final SparkMaxConfig upperWristConfig = new SparkMaxConfig();
  private static SparkMax m_upperWristMotor;

  /** Creates a new Elevator. */
  public UpperWrist(int upperWristMotorDeviceId) {
    m_upperWristMotor = new SparkMax(upperWristMotorDeviceId, MotorType.kBrushless);
    upperWristConfig.idleMode(IdleMode.kBrake);
  }

  public void actuate(double value){
    m_upperWristMotor.set(value);
  }

  public void stop(){
    m_upperWristMotor.set(0);
  }

  public double getEncoder(){
    return m_upperWristMotor.getEncoder().getPosition();
  }

  public void zeroEncoder() {
    m_upperWristMotor.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
      SmartDashboard.putNumber("Upper Wrist Encoder", getEncoder());
  }
}
