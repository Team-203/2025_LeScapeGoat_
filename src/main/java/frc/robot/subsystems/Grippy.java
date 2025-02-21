// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Grippy extends SubsystemBase {

    private static SparkMax m_gripper1;
    private static SparkMax m_gripper2;
    private final SparkMaxConfig gripper1Config = new SparkMaxConfig();
    private final SparkMaxConfig gripper2Config = new SparkMaxConfig();
  /** Creates a new Grippy. */

  public Grippy(int gripperDevice1Id, int gripperDevice2Id) {
    m_gripper1 = new SparkMax(gripperDevice1Id, MotorType.kBrushless);
    m_gripper2 = new SparkMax(gripperDevice2Id, MotorType.kBrushless);
    gripper1Config.idleMode(IdleMode.kBrake);
    gripper2Config.idleMode(IdleMode.kBrake);
  }

  public void intake(double value) {
    m_gripper1.set(value);
    m_gripper2.set(-value);
  }

  public void stop() {
    m_gripper1.set(0);
    m_gripper2.set(0);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
