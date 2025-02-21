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

public class UpperIntake extends SubsystemBase {

  private final SparkMaxConfig upperIntakeConfig = new SparkMaxConfig();
  private static SparkMax m_upperIntakeMotor;

  /** Creates a new Elevator. */
  public UpperIntake(int upperIntakeMotorDeviceId) {
    m_upperIntakeMotor = new SparkMax(upperIntakeMotorDeviceId, MotorType.kBrushless);
    upperIntakeConfig.idleMode(IdleMode.kBrake);
  }
  public void intake(double value){
    m_upperIntakeMotor.set(value);
  }
  public void stop(){
    m_upperIntakeMotor.set(0);
  }
  public double getEncoder(){
    return m_upperIntakeMotor.getEncoder().getPosition();
  }
  public void zeroEncoder() {
    m_upperIntakeMotor.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Upper Intake Encoder", getEncoder());
  }
}
