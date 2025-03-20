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

    public TargetingUtil(Limelight limelightSubsystem, double setpoint, double tolerance, PIDController pidController) {
        this.limelightSubsystem = limelightSubsystem;
        this.pidController = pidController; 
        pidController.setSetpoint(setpoint);
        pidController.setTolerance(tolerance);
    }

    public double calculateRotation() {
      return MathUtil.clamp(pidController.calculate(limelightSubsystem.getTx()),-1,1);
    }

    public boolean onTarget() {
        return pidController.atSetpoint();
    }
    
}