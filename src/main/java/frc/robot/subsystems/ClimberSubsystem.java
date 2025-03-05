// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ClimberSubsystemConstants;

public class ClimberSubsystem extends SubsystemBase {
  // Initialize climber SPARK. We will use MAXMotion position control for the climber, so we also need to
  // initialize the closed loop controller and encoder.
  private SparkFlex climberMotor =
      new SparkFlex(ClimberSubsystemConstants.kClimberMotorCanID, MotorType.kBrushless);
  private SparkClosedLoopController climberController = climberMotor.getClosedLoopController();
  private RelativeEncoder climberEncoder = climberMotor.getEncoder();

  // Member variables for subsystem state management
  private boolean wasReset = false;

  public ClimberSubsystem() {
    /*
     * Apply the configuration to the SPARKs.
     *
     * kResetSafeParameters is used to get the SPARK to a known state. This
     * is useful in case the SPARK is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    climberMotor.configure(
        Configs.ClimberSubsystem.climberConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Zero climber encoder on initialization
    climberEncoder.setPosition(0);
  }

  /** Zero the climber encoder when the user button is pressed on the roboRIO */
  private void zeroOnUserButton() {
    if (!wasReset && RobotController.getUserButton()) {
      // Zero the encoder only when button switches from "unpressed" to "pressed" to prevent
      // constant zeroing while pressed
      wasReset = true;
      climberEncoder.setPosition(0);
    } else if (!RobotController.getUserButton()) {
      wasReset = false;
    }
  }

  /**
   * Command to run the climber. This will extend the climber to its "down" position and run the
   * motor at its "forward" power to raise the robot.
   */
  public Command runClimberCommand() {
    return this.run(
        () -> {
          setClimberPower(ClimberSubsystemConstants.ClimberSetpoints.kForward);
        //   setClimberPosition(ClimberSubsystemConstants.ClimberSetpoints.kDown);
        });
  }

  /**
   * Command to run the climber in reverse. This will extend the climber to its "hold" position and
   * run the motor at its "reverse" power to eject the ball.
   */
  public Command reverseClimberCommand() {
    return this.run(
        () -> {
          setClimberPower(ClimberSubsystemConstants.ClimberSetpoints.kReverse);
          setClimberPosition(ClimberSubsystemConstants.ClimberSetpoints.kHold);
        });
  }

  /** Set the climber motor power in the range of [-1, 1]. */
  private void setClimberPower(double power) {
    climberMotor.set(-power);
  }

  /** Set the arm motor position. This will use closed loop position control. */
  private void setClimberPosition(double position) {
    // climberController.setReference(position, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    zeroOnUserButton();

    // Display subsystem values
    SmartDashboard.putNumber("Climber/Climber/Position", climberEncoder.getPosition());
  }
}
