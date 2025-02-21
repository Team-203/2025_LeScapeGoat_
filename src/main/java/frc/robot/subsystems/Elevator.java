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

public class Elevator extends SubsystemBase {

  private final SparkMaxConfig elevatorConfig = new SparkMaxConfig();
  private static SparkMax m_elevatorMotor;

  /** Creates a new Elevator. */
  public Elevator(int elevatorMotorDeviceId) {
    m_elevatorMotor = new SparkMax(elevatorMotorDeviceId, MotorType.kBrushless);
    elevatorConfig.idleMode(IdleMode.kBrake);
  }
  public void verticalMove(double value){
    m_elevatorMotor.set(value);
  }
  public void stop(){
    m_elevatorMotor.set(0);
  }
  public double getEncoder(){
    return m_elevatorMotor.getEncoder().getPosition();
  }
  public void zeroEncoder() {
    m_elevatorMotor.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Encoder", getEncoder());
  }

}
