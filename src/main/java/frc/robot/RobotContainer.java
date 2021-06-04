package frc.robot;

import static edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.*;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.*;

import java.io.IOException;
import java.nio.file.Paths;

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
    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

    Trajectory barrelsTrajectory;
    Trajectory bounceSeg1Trajectory;
    Trajectory bounceSeg2Trajectory;
    Trajectory bounceSeg3Trajectory;
    Trajectory bounceSeg4Trajectory;
    Trajectory slalomTrajectory;

    Command barrelsCommand;
    Command bounceCommand;
    Command slalomCommand;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        // Set the default drive command to split-stick arcade drive
        m_robotDrive.setDefaultCommand(
                // A split-stick arcade command, with forward/backward controlled by the left
                // hand, and turning controlled by the right.
                /*
                new RunCommand(
                        () ->
                                m_robotDrive.arcadeDrive(
                                        m_driverController.getY(GenericHID.Hand.kLeft),
                                        m_driverController.getX(GenericHID.Hand.kRight) * -1),
                        m_robotDrive));

                 */
                new RunCommand(
                        () ->
                                m_robotDrive.tankDrive(
                                        m_driverController.getY(GenericHID.Hand.kLeft),
                                        m_driverController.getY(GenericHID.Hand.kRight)),
                        m_robotDrive));

        /*
        try {
            barrelsTrajectory = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/paths/barrels.wpilib.json"));
            bounceSeg1Trajectory = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/paths/bounceSeg1.wpilib.json"));
            bounceSeg2Trajectory = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/paths/bounceSeg2.wpilib.json"));
            bounceSeg3Trajectory = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/paths/bounceSeg3.wpilib.json"));
            bounceSeg4Trajectory = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/paths/bounceSeg4.wpilib.json"));
            slalomTrajectory = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/paths/slalom.wpilib.json"));

            barrelsCommand = new SequentialCommandGroup(
                    new InstantCommand(() -> {
                        m_robotDrive.resetOdometry(barrelsTrajectory.getInitialPose());
                    }),
                    new RamseteCommand(
                            barrelsTrajectory,
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
                            m_robotDrive),
                    new InstantCommand(() -> {
                        m_robotDrive.tankDriveVolts(0, 0);
                    })
            );

            bounceCommand = new SequentialCommandGroup(
                    new InstantCommand(() -> {
                        m_robotDrive.resetOdometry(bounceSeg1Trajectory.getInitialPose());
                    }),
                    new RamseteCommand(
                            bounceSeg1Trajectory,
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
                            m_robotDrive),
                    new RamseteCommand(
                            bounceSeg2Trajectory,
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
                            m_robotDrive),
                    new RamseteCommand(
                            bounceSeg3Trajectory,
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
                            m_robotDrive),
                    new RamseteCommand(
                            bounceSeg4Trajectory,
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
                            m_robotDrive),
                    new InstantCommand(() -> {
                        m_robotDrive.tankDriveVolts(0, 0);
                    })
            );

            slalomCommand = new SequentialCommandGroup(
                    new InstantCommand(() -> {
                        m_robotDrive.resetOdometry(slalomTrajectory.getInitialPose());
                    }),
                    new RamseteCommand(
                            slalomTrajectory,
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
                            m_robotDrive),
                    new InstantCommand(() -> {
                        m_robotDrive.tankDriveVolts(0, 0);
                    })
        );

        } catch (IOException ioe) {
            DriverStation.reportError("Unable to open trajectory!", ioe.getStackTrace());
        }

         */

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

        return null;

        //return barrelsCommand;

        //return bounceCommand;

        //return slalomCommand;

        /*

        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint =
                new DifferentialDriveVoltageConstraint(
                        new SimpleMotorFeedforward(
                                DriveConstants.ksVolts,
                                DriveConstants.kvVoltSecondsPerMeter,
                                DriveConstants.kaVoltSecondsSquaredPerMeter),
                        DriveConstants.kDriveKinematics,
                        10);

        // Create config for trajectory
        TrajectoryConfig config =
                new TrajectoryConfig(
                        AutoConstants.kMaxSpeedMetersPerSecond,
                        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(DriveConstants.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
                TrajectoryGenerator.generateTrajectory(
                        // Start at the origin facing the +X direction
                        new Pose2d(0, 0, new Rotation2d(0)),
                        // Pass through these two interior waypoints, making an 's' curve path
                        List.of(new Translation2d(2, 2), new Translation2d(4, -2)),
                        // End 3 meters straight ahead of where we started, facing forward
                        new Pose2d(6, 0, new Rotation2d(0)),
                        // Pass config
                        config);

        RamseteCommand ramseteCommand =
                new RamseteCommand(
                        exampleTrajectory,
                        m_robotDrive::getPose,
                        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
                        new SimpleMotorFeedforward(
                                DriveConstants.ksVolts,
                                DriveConstants.kvVoltSecondsPerMeter,
                                DriveConstants.kaVoltSecondsSquaredPerMeter),
                        DriveConstants.kDriveKinematics,
                        m_robotDrive::getWheelSpeeds,
                        new PIDController(DriveConstants.kPDriveVel, 0, 0),
                        new PIDController(DriveConstants.kPDriveVel, 0, 0),
                        // RamseteCommand passes volts to the callback
                        m_robotDrive::tankDriveVolts,
                        m_robotDrive);

        m_robotDrive.zeroHeading();
        m_robotDrive.resetEncoders();

        // Reset odometry to the starting pose of the trajectory.
        m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));


         */
    }
}