package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.sensors.*;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DriveSubsystem extends SubsystemBase {
    // The motors on the left side of the drive.
    private final WPI_TalonSRX leftMotor1 = new WPI_TalonSRX(DriveConstants.kLeftMotor1Port);
    private final WPI_TalonSRX stickbotGyro = leftMotor1;
    private final SpeedControllerGroup m_leftMotors =
            new SpeedControllerGroup(
                    leftMotor1,
                    new WPI_TalonSRX(DriveConstants.kLeftMotor2Port));

    // The motors on the right side of the drive.
    private final WPI_TalonSRX rightMotor1 = new WPI_TalonSRX(DriveConstants.kRightMotor1Port);
    private final WPI_TalonSRX productionBotGyro = rightMotor1;
    private final SpeedControllerGroup m_rightMotors =
            new SpeedControllerGroup(
                    rightMotor1,
                    new WPI_TalonSRX(DriveConstants.kRightMotor2Port));

    // The robot's drive
    private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

    // The left-side drive encoder
    private final Encoder m_leftEncoder =
            new Encoder(
                    DriveConstants.kLeftEncoderPorts[0],
                    DriveConstants.kLeftEncoderPorts[1],
                    DriveConstants.kLeftEncoderReversed);

    // The right-side drive encoder
    private final Encoder m_rightEncoder =
            new Encoder(
                    DriveConstants.kRightEncoderPorts[0],
                    DriveConstants.kRightEncoderPorts[1],
                    DriveConstants.kRightEncoderReversed);

    // The gyro sensor
    private final PigeonIMU m_gyro = new PigeonIMU(stickbotGyro);

    // Odometry class for tracking robot pose
    private final DifferentialDriveOdometry m_odometry;

    /*
    private final PhotonCamera rustyCam = new PhotonCamera("RustyCam");
    static final double kCameraHeight = 0.3; // meters
    static final double kCameraPitch = 0.0; // radians
    static final double kTargetHeight = 0.0; // meters

     */

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {
        m_leftMotors.setInverted(true);
        m_rightMotors.setInverted(true);
        // Sets the distance per pulse for the encoders
        m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
        m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

        resetEncoders();
        m_odometry = new DifferentialDriveOdometry(getRotation2d());

    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        m_odometry.update(getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());

        SmartDashboard.putNumber("Heading", getHeading());
        SmartDashboard.putNumber("leftEncoder", m_leftEncoder.getDistance());
        SmartDashboard.putNumber("rightEncoder", m_rightEncoder.getDistance());

        /*
        PhotonPipelineResult result = rustyCam.getLatestResult();
        if (result.hasTargets()) {
            SmartDashboard.putNumber("Tracked Targets", result.getTargets().size());
            PhotonTrackedTarget target1 = result.getTargets().get(0);
            double distanceMeters = PhotonUtils.calculateDistanceToTargetMeters(
                    kCameraHeight, kTargetHeight, kCameraPitch, Math.toRadians(target1.getPitch()));
            SmartDashboard.putString("Target1", "Distance[" + distanceMeters + "]");
            if (result.getTargets().size() > 1) {
                PhotonTrackedTarget target2 = result.getTargets().get(1);
                double distanceMeters2 = PhotonUtils.calculateDistanceToTargetMeters(
                        kCameraHeight, kTargetHeight, kCameraPitch, Math.toRadians(target2.getPitch()));
                SmartDashboard.putString("Target2", "Distance[" + distanceMeters2 + "]");
            }
            if (result.getTargets().size() > 2) {
                PhotonTrackedTarget target3 = result.getTargets().get(2);
                double distanceMeters3 = PhotonUtils.calculateDistanceToTargetMeters(
                        kCameraHeight, kTargetHeight, kCameraPitch, Math.toRadians(target3.getPitch()));
                SmartDashboard.putString("Target3", "Distance[" + distanceMeters3 + "]");
            }
        }

         */
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(pose, getRotation2d());
    }

    /**
     * Drives the robot using arcade controls.
     *
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */
    public void arcadeDrive(double fwd, double rot) {
        m_drive.arcadeDrive(fwd, rot);
    }


    public void tankDrive(double left, double right) {
        m_drive.tankDrive(left, right);
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     *
     * @param leftVolts the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        m_leftMotors.setVoltage(-leftVolts);
        m_rightMotors.setVoltage(rightVolts);
        m_drive.feed();
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        m_leftEncoder.reset();
        m_rightEncoder.reset();
    }

    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistance() {
        return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
    }

    /**
     * Gets the left drive encoder.
     *
     * @return the left drive encoder
     */
    public Encoder getLeftEncoder() {
        return m_leftEncoder;
    }

    /**
     * Gets the right drive encoder.
     *
     * @return the right drive encoder
     */
    public Encoder getRightEncoder() {
        return m_rightEncoder;
    }

    /**
     * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
        m_drive.setMaxOutput(maxOutput);
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        m_gyro.setFusedHeading(0);
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return m_gyro.getFusedHeading();
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

}