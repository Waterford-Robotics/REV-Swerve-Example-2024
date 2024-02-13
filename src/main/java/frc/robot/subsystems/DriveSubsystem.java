// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

// Drive Subsystem class wahuuu
public class DriveSubsystem extends SubsystemBase {

  // Creates MAX Swerve Modules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
    DriveConstants.kFrontLeftDrivingCanId,
    DriveConstants.kFrontLeftTurningCanId,
    DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
    DriveConstants.kFrontRightDrivingCanId,
    DriveConstants.kFrontRightTurningCanId,
    DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_backLeft = new MAXSwerveModule(
    DriveConstants.kRearLeftDrivingCanId,
    DriveConstants.kRearLeftTurningCanId,
    DriveConstants.kBackLeftChassisAngularOffset);
  
  private final MAXSwerveModule m_backRight = new MAXSwerveModule(
    DriveConstants.kRearRightDrivingCanId,
    DriveConstants.kRearRightTurningCanId,
    DriveConstants.kBackRightChassisAngularOffset);

  // Creates the Gyro for Swerve Magic
  private final AHRS m_gyro = new AHRS();

  // Slew Rate Variables & Objects - Change of Voltage per microsecond
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magRateLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotRateLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  // Odometry - Tracks robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
    DriveConstants.kDriveKinematics, 
    Rotation2d.fromDegrees(m_gyro.getYaw()), 
    new SwerveModulePosition[] {
    m_frontLeft.getPosition(),
    m_frontRight.getPosition(),
    m_backLeft.getPosition(),
    m_backRight.getPosition()
    });

  // Creates a new Drive Subsystem
  public DriveSubsystem() {
  }
  
  @Override
  // This method will be called once per scheduler run
  // Periodically update the odometry
  public void periodic() {
    m_odometry.update(
      Rotation2d.fromDegrees(m_gyro.getYaw()), 
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_backLeft.getPosition(),
        m_backRight.getPosition()
      });

      // Puts Yaw + Angle on Smart Dashboard
      SmartDashboard.putNumber("NavX Yaw", -m_gyro.getYaw());
      SmartDashboard.putNumber("NavX Angle", m_gyro.getAngle());
  }

  // Returns currently estimated pose of robot
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  // Resets odometry to specified pose
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
      Rotation2d.fromDegrees(-m_gyro.getYaw()), 
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_backLeft.getPosition(),
        m_backRight.getPosition()
      }, 
      pose);
  }

   // Method to drive the robot using joystick info
   // xSpeed         Speed of the robot in the x direction (forward).
   // ySpeed         Speed of the robot in the y direction (sideways).
   // rot            Angular rate of the robot.
   // fieldRelative  Whether the provided x and y speeds are relative to the field.
   // rateLimit      Whether to enable rate limiting for smoother control.
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

    // x and y commanded speed variables
    double xSpeedCommanded;
    double ySpeedCommanded;

    // Checks for rate limiting
    if (rateLimit) {

      // Converts Cartesian XY to Polar for Rate Limiting
      // Very interesting calculus topic if you're interested!
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate direction slew rate based on lateral accel estimate
      double directionSlewRate;

      // If the translation magnitude isn't zero, calculate the slew rate
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      }

      // Otherwise, set the rate to a high number (almost instantaneous)
      else {
        directionSlewRate = 500.0;
      }

      // Ensures a motor doesn't have to turn more than 90 degrees to
      // get to any position desired by the driver
      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
    }
  }



  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
