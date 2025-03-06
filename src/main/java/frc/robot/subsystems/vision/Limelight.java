// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  
  private NetworkTable limelight;
  private NetworkTableEntry NetworkTx;
  private NetworkTableEntry NetworkTy;
  private NetworkTableEntry NetworkTa;
  
  
  /** Creates a new LimelightSubsystem. */
  //Default constructor that assumes limelight name as "limelight" (allows earlier code to work)
  public Limelight() {
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTx = limelight.getEntry("tx");
    NetworkTy = limelight.getEntry("ty");
    NetworkTa = limelight.getEntry("ta");
  }

  public Limelight(String limelightName) {
    limelight = NetworkTableInstance.getDefault().getTable(limelightName);
    NetworkTx = limelight.getEntry("tx");
    NetworkTy = limelight.getEntry("ty");
    NetworkTa = limelight.getEntry("ta");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Limelight X", this.GetTx());
  }

// pipeline funnies
  public void switchPipe(int pipe){
    limelight.getEntry("pipeline").setValue(pipe);
  }

  public double getPipe(){
    return limelight.getEntry("getpipe").getDouble(0);
    //slap into shuffle board wink wonk?
  }


// object tracking vars

  public double GetTx(){
    return NetworkTx.getDouble(0.0);
  }

  public double GetTy(){
    return NetworkTy.getDouble(0.0);
  }
  
  public double GetTa(){
    return NetworkTa.getDouble(0.0);
  }


// April Tag Vars
  public double GetX() {
    return limelight.getEntry("botpose").getDoubleArray(new double[6])[0];
  }

  public double GetY(){
    return limelight.getEntry("botpose").getDoubleArray(new double[6])[1];
  }

  public double GetZ(){
    return limelight.getEntry("botpose").getDoubleArray(new double[6])[3];
  }

  public double GetPitch(){
    return limelight.getEntry("botpose").getDoubleArray(new double[6])[4];
  }

  public double GetRoll(){
    return limelight.getEntry("botpose").getDoubleArray(new double[6])[5];
  }

  public double GetYaw(){
    return limelight.getEntry("botpose").getDoubleArray(new double[6])[6];
  }
 
}