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

public class Climber extends SubsystemBase {

  private final SparkMaxConfig climberConfig = new SparkMaxConfig();
  private static SparkMax m_climberMotor;

  /** Creates a new Elevator. */
  public Climber(int climberMotorDeviceId) {
    m_climberMotor = new SparkMax(climberMotorDeviceId, MotorType.kBrushless);
    climberConfig.idleMode(IdleMode.kBrake);
  }
  public void climb(double value){
    m_climberMotor.set(value);
  }
  public void stop(){
    m_climberMotor.set(0);
  }
  public double getEncoder(){
    return m_climberMotor.getEncoder().getPosition();
  }
  public void zeroEncoder() {
    m_climberMotor.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber Encoder", getEncoder());
  }
}
