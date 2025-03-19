// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.CoralSubsystem.Setpoint;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.utils.TargetingUtil;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
private final Limelight m_limelight = new Limelight();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(new TargetingUtil(m_limelight));
  private final CoralSubsystem m_coralSubSystem = new CoralSubsystem();
  private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();

  // The driver's controller
  CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);
  // The operator's controller
  CommandXboxController m_operatorController =
      new CommandXboxController(OIConstants.kOperatorControllerPort);

      // Autonomous Chooser
  private SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    true, m_driverController.getHID().getLeftTriggerAxis() > 0.2, m_driverController.getHID().getRightTriggerAxis() > 0.2),
            m_robotDrive));

    // Set the ball intake to in/out when not running based on internal state
    m_algaeSubsystem.setDefaultCommand(m_algaeSubsystem.idleCommand());

    //Auto Configurations
    NamedCommands.registerCommand("MoveToL2", m_coralSubSystem.setSetpointCommand(Setpoint.kLevel2));
    NamedCommands.registerCommand("Shoot", m_coralSubSystem.reverseIntakeCommand());

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Operator controls
    // - Set swerve to X
    // m_driverController.leftStick().whileTrue(m_robotDrive.setXCommand());

    // Zero Heading
    m_driverController.start().onTrue(new InstantCommand(
        () -> m_robotDrive.zeroHeading(), m_robotDrive
    ));
    // - Jog L/R
    m_driverController.leftBumper().whileTrue(new RunCommand(
        () -> m_robotDrive.drive(0, 0.2, 0, false, false, false)));
    m_driverController.rightBumper().whileTrue(new RunCommand(
        () -> m_robotDrive.drive(0, -0.2, 0, false, false, false)));


    m_driverController.povUp().whileTrue(m_climberSubsystem.runClimberCommand());

    m_driverController.povDown().whileTrue(m_climberSubsystem.reverseClimberCommand());
    // Operator controls

    // - Run coral intake
    m_operatorController.leftBumper().whileTrue(m_coralSubSystem.runIntakeCommand());

    // - Run coral outtake
    m_operatorController.rightBumper().whileTrue(m_coralSubSystem.reverseIntakeCommand());


    // - Elevator/Arm to human player position, set ball intake to stow when idle
    m_operatorController.b().onTrue(
        m_coralSubSystem
            .setSetpointCommand(Setpoint.kFeederStation)
            .alongWith(m_algaeSubsystem.stowCommand())
    );

    // - Elevator/Arm to level 2 position
    m_operatorController.a().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel2));

    // - Elevator/Arm to level 3 position
    m_operatorController.y().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel3));


    // - Run algae intake, set to leave out when idle
    m_operatorController
        .leftTrigger(OIConstants.kTriggerButtonThreshold)
        .whileTrue(m_algaeSubsystem.runIntakeCommand());

    // - Run algae intake in reverse, set to stow when idle
    m_operatorController
        .rightTrigger(OIConstants.kTriggerButtonThreshold)
        .whileTrue(m_algaeSubsystem.reverseIntakeCommand());

    // - Run algae intake, set to leave out when idle
    m_operatorController
        .povUp()
        .whileTrue(m_algaeSubsystem.runIntakeCoralCommand());

    // // Wrist Adjust Up
    // m_operatorController.povUp().onTrue(new InstantCommand(() -> m_coralSubSystem.setWristPower(0.1), m_coralSubSystem)).onFalse(new InstantCommand(() -> m_coralSubSystem.setWristPower(0), m_coralSubSystem));

    // // Wrist Adjust Down
    // m_operatorController.povDown().onTrue(new InstantCommand(() -> m_coralSubSystem.setWristPower(-0.1), m_coralSubSystem)).onFalse(new InstantCommand(() -> m_coralSubSystem.setWristPower(0), m_coralSubSystem));


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return autoChooser.getSelected();

    // return null;



    // // Create config for trajectory
    // TrajectoryConfig config =
    //     new TrajectoryConfig(
    //             AutoConstants.kMaxSpeedMetersPerSecond,
    //             AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //         // Add kinematics to ensure max speed is actually obeyed
    //         .setKinematics(DriveConstants.kDriveKinematics);

    // // An example trajectory to follow. All units in meters.
    // Trajectory exampleTrajectory =
    //     TrajectoryGenerator.generateTrajectory(
    //         // Start at the origin facing the +X direction
    //         new Pose2d(0, 0, new Rotation2d(0)),
    //         // Pass through these two interior waypoints, making an 's' curve path
    //         List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //         // End 3 meters straight ahead of where we started, facing forward
    //         new Pose2d(3, 0, new Rotation2d(0)),
    //         config);

    // var thetaController =
    //     new ProfiledPIDController(
    //         AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // SwerveControllerCommand swerveControllerCommand =
    //     new SwerveControllerCommand(
    //         exampleTrajectory,
    //         m_robotDrive::getPose, // Functional interface to feed supplier
    //         DriveConstants.kDriveKinematics,

    //         // Position controllers
    //         new PIDController(AutoConstants.kPXController, 0, 0),
    //         new PIDController(AutoConstants.kPYController, 0, 0),
    //         thetaController,
    //         m_robotDrive::setModuleStates,
    //         m_robotDrive);

    // // Reset odometry to the starting pose of the trajectory.
    // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // // Run path following command, then stop at the end.
    // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false, false));
  }
}
