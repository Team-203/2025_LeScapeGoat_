// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LimelightHelpers;

public class Limelight extends SubsystemBase {
  
  private String limelightName;
  
  /** Creates a new Limelight */
  public Limelight(String limelightName) {
    this.limelightName = limelightName;
  }

  // Default constructor that assumes limelight name as "limelight"
  public Limelight() {
    this("limelight");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Limelight TX", this.getTx());
  }

  // Object Tracking Variables
  public double getTx(){
    return LimelightHelpers.getTX(limelightName);
  }

  public double getTy(){
    return LimelightHelpers.getTY(limelightName);
  }
  
  public double getTa(){
    return LimelightHelpers.getTA(limelightName);
  }
 
}