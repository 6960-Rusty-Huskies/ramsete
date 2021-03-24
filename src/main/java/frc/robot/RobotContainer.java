package frc.robot;

import static edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import frc.robot.Constants.*;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.*;

import java.io.IOException;
import java.nio.file.Paths;
import java.util.List;

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

    Trajectory trajectory;

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

        try {
            trajectory = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/paths/barrels.wpilib.json"));
        } catch (IOException ioe) {
            DriverStation.reportError("Unable to open trajectory!", ioe.getStackTrace());
        }

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