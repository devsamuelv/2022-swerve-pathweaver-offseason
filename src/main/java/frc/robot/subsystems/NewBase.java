// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper.GearRatio;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class NewBase extends SubsystemBase {
  private AHRS gyro = new AHRS();

  private static final double MAX_VOLTAGE = 12.0;
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.14528;
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND
      / Math.hypot(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0);

  private final SwerveModule frontLeftModule;
  private final SwerveModule frontRightModule;
  private final SwerveModule backLeftModule;
  private final SwerveModule backRightModule;

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
      new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
      new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
      new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0));

  public NewBase() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    this.backLeftModule = Mk3SwerveModuleHelper.createFalcon500(
        tab.getLayout("back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0), GearRatio.STANDARD,
        9, 2, Constants.backLeftCANCoderId, Constants.BACK_LEFT_MODULE_STEER_OFFSET);

    this.backRightModule = Mk3SwerveModuleHelper.createFalcon500(
        tab.getLayout("back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0), GearRatio.STANDARD,
        4, 6, Constants.backRightCANCoderId, Constants.BACK_RIGHT_MODULE_STEER_OFFSET);
    this.frontLeftModule = Mk3SwerveModuleHelper.createFalcon500(
        tab.getLayout("front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0), GearRatio.STANDARD,
        3, 7, Constants.frontLeftCANCoderId, Constants.FRONT_LEFT_MODULE_STEER_OFFSET);

    this.frontRightModule = Mk3SwerveModuleHelper.createFalcon500(
        tab.getLayout("front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0), GearRatio.STANDARD,
        8, 5, Constants.frontRightCANCoderId, Constants.FRONT_RIGHT_MODULE_STEER_OFFSET);

  }

  public void zeroGyro() {
    gyro.reset();
  }

  public Rotation2d getRotation() {
    return Rotation2d.fromDegrees(gyro.getFusedHeading());
  }

  public boolean isInverted() {
    return getRotation().getDegrees() <= 190 && getRotation().getDegrees() > 90
        || getRotation().getDegrees() >= 290 && getRotation().getDegrees() < 90;
  }

  public double getDirection() {
    if (isInverted()) {
      return 1.0;
    }

    return -1.0;
  }

  public void drive(ChassisSpeeds speeds) {
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

    frontLeftModule.set((states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE) * -1.0,
        states[0].angle.getRadians());
    frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
        states[1].angle.getRadians());
    backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
        states[2].angle.getRadians());
    backRightModule.set((states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE) * -1.0,
        states[3].angle.getRadians());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
