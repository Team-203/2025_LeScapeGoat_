// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Grippy;
import frc.robot.subsystems.LowerWrist;
import frc.robot.subsystems.UpperIntake;
import frc.robot.subsystems.UpperWrist;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.utils.TargetingUtil;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Climber m_climber = new Climber(SubsystemConstants.kClimberDeviceId);
  private final Elevator m_elevator = new Elevator(SubsystemConstants.kElevatorDeviceId);
  private final Grippy m_grippy = new Grippy(SubsystemConstants.kGrippyDevice1Id, SubsystemConstants.kGrippyDevice2Id);
  private final LowerWrist m_lowerWrist = new LowerWrist(Constants.SubsystemConstants.kLowerWristDeviceId);
  private final UpperWrist m_upperWrist = new UpperWrist(Constants.SubsystemConstants.kUpperWristDeviceId);
  private final UpperIntake m_upperIntake = new UpperIntake(Constants.SubsystemConstants.kUpperIntakeDeviceId);  
  private final Limelight m_limelight = new Limelight();
  private final SwerveSubsystem m_robotDrive = new SwerveSubsystem(new TargetingUtil(m_limelight));



  private final CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

  // Autonomous Chooser
  // private SendableChooser<Command> autoChooser;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    m_robotDrive.setDefaultCommand(
      new RunCommand(
        () -> m_robotDrive.drive(
            -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
            true, m_driverController.getHID().getLeftBumperButton(), m_driverController.getHID().getRightBumperButton()),
        m_robotDrive));

    m_elevator.setDefaultCommand(
      new RunCommand(
        () -> m_elevator.verticalMove(
          m_operatorController.getLeftTriggerAxis() - m_operatorController.getRightTriggerAxis()), 
        m_elevator));

    m_climber.setDefaultCommand(
      new RunCommand(
        () -> m_climber.climb(
          m_driverController.getLeftTriggerAxis() - m_driverController.getRightTriggerAxis()), 
        m_climber));

    // autoChooser = AutoBuilder.buildAutoChooser();
    // SmartDashboard.putData("Auto Chooser", autoChooser);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // Spin Upper Intake Out
    m_operatorController.povUp().onTrue(  
    new InstantCommand(
      () -> m_upperIntake.intake(0.3), m_upperIntake
    )).onFalse(new InstantCommand(
      m_upperIntake::stop, m_upperIntake
    ));

    // Spin Upper Intake In
    m_operatorController.povDown().onTrue(  
    new InstantCommand(
      () -> m_upperIntake.intake(-0.3), m_upperIntake
    )).onFalse(new InstantCommand(
      m_upperIntake::stop, m_upperIntake
    ));


    // Spin Lower Wrist Up
    m_operatorController.b().onTrue(  
    new InstantCommand(
      () -> m_lowerWrist.actuate(0.13), m_lowerWrist
    )).onFalse(new InstantCommand(
      m_lowerWrist::stop, m_lowerWrist
    ));

    // Spin Lower Wrist Down
    m_operatorController.x().onTrue(  
    new InstantCommand(
      () -> m_lowerWrist.actuate(-0.13), m_lowerWrist
    )).onFalse(new InstantCommand(
      m_lowerWrist::stop, m_lowerWrist
    ));


    // Spin Upper Wrist Up
    m_operatorController.y().onTrue(  
    new InstantCommand(
      () -> m_upperWrist.actuate(-0.3), m_upperWrist
    )).onFalse(new InstantCommand(
      m_upperWrist::stop, m_upperWrist
    ));

    // Spin Upper Wrist Down
    m_operatorController.a().onTrue(  
    new InstantCommand(
      () -> m_upperWrist.actuate(0.3), m_upperWrist
    )).onFalse(new InstantCommand(
      m_upperWrist::stop, m_upperWrist
    ));


    // Accumulate Grippy In
    m_operatorController.leftBumper().onTrue(  
    new InstantCommand(
      () -> m_grippy.accumulate(-0.3), m_grippy
    )).onFalse(new InstantCommand(
      m_grippy::stop, m_grippy
    ));

    // Accumulate Grippy Out
    m_operatorController.rightBumper().onTrue(  
    new InstantCommand(
      () -> m_grippy.accumulate(0.3), m_grippy
    )).onFalse(new InstantCommand(
      m_grippy::stop, m_grippy
    ));


    m_operatorController.povLeft().onTrue(  
    new InstantCommand(
      () -> m_grippy.accumulateCoral(-0.3), m_grippy
    )).onFalse(new InstantCommand(
      m_grippy::stop, m_grippy
    ));



  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return autoChooser.getSelected();
    return null;
  }
}
