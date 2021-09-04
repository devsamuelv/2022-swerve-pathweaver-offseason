// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// Stoped at https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-odometry.html page of the docs
public class Base extends SubsystemBase {
  double kMaxSpeed = 3.0;

  Translation2d m_frontLeft = new Translation2d(1, -1);
  Translation2d m_frontRight = new Translation2d(1, 1);
  Translation2d m_backLeft = new Translation2d(-1, -1);
  Translation2d m_backRight = new Translation2d(-1, 1);

  AHRS navx = new AHRS();

  CANCoder frontLeftCANCoder = new CANCoder(Constants.frontLeftCANCoderId);
  CANCoder frontRightCANCoder = new CANCoder(Constants.frontRightCANCoderId);
  CANCoder backLeftCANCoder = new CANCoder(Constants.backLeftCANCoderId);
  CANCoder backRightCANCoder = new CANCoder(Constants.backRightCANCoderId);

  /**
   * The order in which you pass in the wheel locations is the same order that you
   * will receive the module states when performing inverse kinematics. It is also
   * expected that you pass in the module states in the same order when calling
   * the forward kinematics methods.
   */
  SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeft, m_frontRight, m_backLeft, m_backRight);
  ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(1, 3, 1.5);

  SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);

  SwerveModuleState frontLeft = states[0];
  SwerveModuleState frontRight = states[1];
  SwerveModuleState backLeft = states[2];
  SwerveModuleState backRight = states[3];

  SwerveModuleState frontLeftOptimzed = SwerveModuleState.optimize(frontLeft, getCANCoderPosition(frontLeftCANCoder));
  SwerveModuleState frontRightOptimzed = SwerveModuleState.optimize(frontRight,
      getCANCoderPosition(frontRightCANCoder));

  SwerveModuleState backLeftOptimzed = SwerveModuleState.optimize(backLeft, getCANCoderPosition(backLeftCANCoder));
  SwerveModuleState backRightOptimzed = SwerveModuleState.optimize(backRight, getCANCoderPosition(backRightCANCoder));

  SwerveDriveOdometry odometry = new SwerveDriveOdometry(m_kinematics, getGyroHeading(),
      new Pose2d(0, 0, new Rotation2d()));

  Pose2d currentPose = new Pose2d();

  /** Creates a new ExampleSubsystem. */
  public Base() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentPose = odometry.update(getGyroHeading(), frontLeftOptimzed, frontRightOptimzed, backLeftOptimzed,
        backRightOptimzed);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  @SuppressWarnings("ParameterName")
  public void Drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, navx.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.normalizeWheelSpeeds(states, kMaxSpeed);
    m_frontRight.setDesiredState(states[1]);
    m_backLeft.setDesiredState(states[2]);
    m_backRight.setDesiredState(states[3]);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public Rotation2d getGyroHeading() {
    return Rotation2d.fromDegrees(navx.getAngle());
  }

  public Rotation2d getCANCoderPosition(CANCoder _CANCoder) {
    return new Rotation2d(Math.toRadians(_CANCoder.getPosition()));
  }
}
