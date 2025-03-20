// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.vision.Limelight;

/** Add your docs here. */
public class TargetingUtil {

    private Limelight limelightSubsystem;
    private PIDController pidController;

    public TargetingUtil(Limelight limelightSubsystem, double tolerance, double p){
        this.limelightSubsystem = limelightSubsystem;
        pidController = new PIDController(p, 0, 0.0005); 
        pidController.setSetpoint(0);
        pidController.setTolerance(tolerance);
    }

    public TargetingUtil(Limelight limelightSubsystem) {
        this(limelightSubsystem, .25, 0.07);
    }

    public double calculateRotation(){
      return -MathUtil.clamp(pidController.calculate(limelightSubsystem.GetTx()),-1,1);

    }

    public boolean onTarget(){
        return pidController.atSetpoint();
    }
}