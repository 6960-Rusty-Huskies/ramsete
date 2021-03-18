// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj.XboxController.Button;
import static frc.robot.Constants.DriveConstants.autoVoltageConstraint;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.trajectory.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.DriveSubsystem;

import java.io.*;
import java.nio.file.*;
import java.util.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();

    // The driver's controller
    XboxController m_driverController = new XboxController(Constants.OIConstants.kDriverControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        // Set the default drive command to split-stick arcade drive
        m_robotDrive.setDefaultCommand(
                // A split-stick arcade command, with forward/backward controlled by the left
                // hand, and turning controlled by the right.
                new RunCommand(
                        () ->
                                m_robotDrive.arcadeDrive(
                                        m_driverController.getY(GenericHID.Hand.kLeft),
                                        m_driverController.getX(GenericHID.Hand.kRight)),
                        m_robotDrive));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        // Drive at half speed when the right bumper is held
        new JoystickButton(m_driverController, Button.kBumperRight.value)
                .whenPressed(() -> m_robotDrive.setMaxOutput(0.5))
                .whenReleased(() -> m_robotDrive.setMaxOutput(1));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        /*
        try {
            Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/paths/barrels.wpilib.json"));

            RamseteCommand ramseteCommand =
                    new RamseteCommand(
                            trajectory,
                            m_robotDrive::getPose,
                            new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
                            new SimpleMotorFeedforward(
                                    Constants.DriveConstants.ksVolts,
                                    Constants.DriveConstants.kvVoltSecondsPerMeter,
                                    Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
                            Constants.DriveConstants.kDriveKinematics,
                            m_robotDrive::getWheelSpeeds,
                            new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
                            new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
                            // RamseteCommand passes volts to the callback
                            m_robotDrive::tankDriveVolts,
                            m_robotDrive);

            // Reset odometry to the starting pose of the trajectory.
            m_robotDrive.resetOdometry(trajectory.getInitialPose());

            // Run path following command, then stop at the end.
            return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
        } catch (IOException ioe) {
            DriverStation.reportError("Unable to open trajectory!", ioe.getStackTrace());
            return null;
        }

         */


        TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(Constants.DriveConstants.kDriveKinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint);

        Trajectory moveForward =
                TrajectoryGenerator.generateTrajectory(
                        // Start at the origin facing the +X direction
                        new Pose2d(0, 0, new Rotation2d(0)),
                        // Pass through these two interior waypoints, making an 's' curve path
                        List.of(new Translation2d(2, 2), new Translation2d(4, -2)),
                        // End 3 meters straight ahead, facing forward
                        new Pose2d(6, 0, new Rotation2d(0)),
                        // Pass config
                        config);

        RamseteCommand ramseteSimpleMoveForwardCommand =
                new RamseteCommand(
                        moveForward,
                        m_robotDrive::getPose,
                        new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
                        new SimpleMotorFeedforward(
                                Constants.DriveConstants.ksVolts,
                                Constants.DriveConstants.kvVoltSecondsPerMeter,
                                Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
                        Constants.DriveConstants.kDriveKinematics,
                        m_robotDrive::getWheelSpeeds,
                        new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
                        new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
                        // RamseteCommand passes volts to the callback
                        m_robotDrive::tankDriveVolts,
                        m_robotDrive);

        return ramseteSimpleMoveForwardCommand;
    }
}