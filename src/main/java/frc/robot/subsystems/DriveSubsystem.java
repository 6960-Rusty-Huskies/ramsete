/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

  private final DifferentialDrive drive;
  private final SpeedControllerGroup leftMotors;
  private final SpeedControllerGroup rightMotors;
  private final Encoder leftEncoder;
  private final Encoder rightEncoder;
  private final PigeonIMU gyro;
  //private final SlewRateLimiter leftRateLimiter = new SlewRateLimiter(.9);
  //private final SlewRateLimiter rightRateLimiter = new SlewRateLimiter(.9);

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry odometry;

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    WPI_TalonSRX leftTalon1 = new WPI_TalonSRX(DriveConstants.kLeftMotor1Port);
    WPI_TalonSRX leftTalon2 = new WPI_TalonSRX(DriveConstants.kLeftMotor2Port);
    leftMotors = new SpeedControllerGroup(leftTalon1, leftTalon2);
    leftMotors.setInverted(true);

    // The motors on the right side of the drive.
    WPI_TalonSRX rightTalon1 = new WPI_TalonSRX(DriveConstants.kRightMotor1Port);
    WPI_TalonSRX rightTalon2 = new WPI_TalonSRX(DriveConstants.kRightMotor2Port);
    rightMotors = new SpeedControllerGroup(rightTalon1, rightTalon2);
    rightMotors.setInverted(true);

    drive = new DifferentialDrive(leftMotors, rightMotors);

    // Sets the distance per pulse for the encoders
    leftEncoder = new Encoder(
            DriveConstants.kLeftEncoderPorts[0],
            DriveConstants.kLeftEncoderPorts[1],
            DriveConstants.kLeftEncoderReversed);
    leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    rightEncoder = new Encoder(
            DriveConstants.kRightEncoderPorts[0],
            DriveConstants.kRightEncoderPorts[1],
            DriveConstants.kRightEncoderReversed);
    rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    resetEncoders();

    gyro = new PigeonIMU(leftTalon1);

    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Encoder", leftEncoder.getDistance());
    SmartDashboard.putNumber("Right Encoder", rightEncoder.getDistance());
    SmartDashboard.putNumber("Average Encoder Distance", getAverageEncoderDistance());

    // Update the odometry in the periodic block
    odometry.update(
            Rotation2d.fromDegrees(getHeading()),
            leftEncoder.getDistance(),
            rightEncoder.getDistance());

  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    drive.arcadeDrive(fwd, rot);
  }

  public void tankDrive(double left, double right) {
    /*
    SmartDashboard.putNumber("tank left", left);
    SmartDashboard.putNumber("tank right", right);
    left = leftRateLimiter.calculate(left);
    right = rightRateLimiter.calculate(right);
    SmartDashboard.putNumber("slew left", left);
    SmartDashboard.putNumber("slew right", right);
     */
    drive.tankDrive(left, right);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    SmartDashboard.putString("Drive Volts", String.format("left[%f] right[%f]", leftVolts, rightVolts));
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(-rightVolts);
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public Encoder getLeftEncoder() {
    return leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public Encoder getRightEncoder() {
    return rightEncoder;
  }

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
    SmartDashboard.putNumber("Heading", gyro.getFusedHeading());
    return gyro.getFusedHeading();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  /*
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
  */
}
