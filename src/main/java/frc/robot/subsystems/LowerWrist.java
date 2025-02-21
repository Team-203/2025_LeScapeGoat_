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

public class LowerWrist extends SubsystemBase {

  private final SparkMaxConfig lowerWristConfig = new SparkMaxConfig();
  private static SparkMax m_lowerWristMotor;

  /** Creates a new Elevator. */
  public LowerWrist(int lowerWristMotorDeviceId) {
    m_lowerWristMotor = new SparkMax(lowerWristMotorDeviceId, MotorType.kBrushless);
    lowerWristConfig.idleMode(IdleMode.kBrake);
  }
  public void actuate(double value){
    m_lowerWristMotor.set(value);
  }
  public void stop(){
    m_lowerWristMotor.set(0);
  }
  public double getEncoder(){
    return m_lowerWristMotor.getEncoder().getPosition();
  }
  public void zeroEncoder() {
    m_lowerWristMotor.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Lower Wrist Encoder", getEncoder());
  }

}
