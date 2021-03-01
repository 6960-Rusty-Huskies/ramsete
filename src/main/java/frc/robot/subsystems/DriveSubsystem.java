// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.
  private final SpeedControllerGroup m_leftMotors;

  // The motors on the right side of the drive.
  private final SpeedControllerGroup m_rightMotors;

  // The robot's drive
  private final DifferentialDrive m_drive;

  // The left-side drive encoder
  private final Encoder m_leftEncoder =
          new Encoder(
                  Constants.DriveConstants.kLeftEncoderPorts[0],
                  Constants.DriveConstants.kLeftEncoderPorts[1],
                  Constants.DriveConstants.kLeftEncoderReversed);

  // The right-side drive encoder
  private final Encoder m_rightEncoder =
          new Encoder(
                  Constants.DriveConstants.kRightEncoderPorts[0],
                  Constants.DriveConstants.kRightEncoderPorts[1],
                  Constants.DriveConstants.kRightEncoderReversed);

  // The gyro sensor
  private final PigeonIMU m_gyro;

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    WPI_TalonSRX leftMotor1 = new WPI_TalonSRX(Constants.DriveConstants.kLeftMotor1Port);
    WPI_TalonSRX leftMotor2 = new WPI_TalonSRX(Constants.DriveConstants.kLeftMotor2Port);
    m_leftMotors = new SpeedControllerGroup(leftMotor1, leftMotor2);
    m_leftMotors.setInverted(true);

    WPI_TalonSRX rightMotor1 = new WPI_TalonSRX(Constants.DriveConstants.kRightMotor1Port);
    WPI_TalonSRX rightMotor2 = new WPI_TalonSRX(Constants.DriveConstants.kRightMotor2Port);
    m_rightMotors = new SpeedControllerGroup(rightMotor1, rightMotor2);
    m_rightMotors.setInverted(true);

    m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

    // Sets the distance per pulse for the encoders
    m_leftEncoder.setDistancePerPulse(Constants.DriveConstants.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(Constants.DriveConstants.kEncoderDistancePerPulse);
    resetEncoders();

    m_gyro = new PigeonIMU(leftMotor1);

    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
            Rotation2d.fromDegrees(getHeading()),
            m_leftEncoder.getDistance(),
            m_rightEncoder.getDistance());

    SmartDashboard.putNumber("Left Encoder", m_leftEncoder.getDistance());
    SmartDashboard.putNumber("Right Encoder", m_rightEncoder.getDistance());
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
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot * -1);
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
  /*
  public void zeroHeading() {
    m_gyro.reset();
  }

   */

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    SmartDashboard.putNumber("Heading", m_gyro.getFusedHeading());
    return m_gyro.getFusedHeading();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  /*
  public double getTurnRate() {
    return -m_gyro.getRate();
  }

   */
}