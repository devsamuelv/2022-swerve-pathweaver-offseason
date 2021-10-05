// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.modules;

import javax.annotation.Nullable;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.hal.EncoderJNI;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.modules.Talon;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import com.swervedrivespecialties.swervelib.ctre.*;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.Constants;

/** Add your docs here. */
public class SwervePod {
  double kWheelRadius = 2;
  int kEncoderResolution = 2048;

  double kModuleMaxAngularVelocity = Constants.kMaxAngularSpeed;
  double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

  // tune the pid values for the robot
  // (0.1, 0.0, 20.0, 1023.0 / 6800.0, 300, 0.50)
  PIDController m_drivePIDController = new PIDController(50, 0, 0);
  ProfiledPIDController m_turningPIDController = new ProfiledPIDController(1, 0.0, 0.0,
      new TrapezoidProfile.Constraints(kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0, 0, 0.2);
  SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(0, 0, 0.2);

  SpeedTalon speedMotor;
  AngleTalon angularMotor;
  CANCoder canCoder;

  public void setPID(double kP, double kI, double kD) {
    m_drivePIDController.setPID(kP, kI, kD);
  }

  public SwervePod(int speedMotorId, int angularMotorId, int canCoderId) {
    this.speedMotor = new SpeedTalon(speedMotorId);
    this.angularMotor = new AngleTalon(angularMotorId);
    this.canCoder = new CANCoder(canCoderId);

    this.canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

    // this.m_driveFeedforward.maxAchievableAcceleration(12, 100);
    this.m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public SwervePod(int speedMotorId, int angularMotorId, int canCoderId, double kWheelRadius, int kEncoderResolution) {
    this.speedMotor = new SpeedTalon(speedMotorId);
    this.angularMotor = new AngleTalon(angularMotorId);
    this.canCoder = new CANCoder(canCoderId);

    this.kWheelRadius = kWheelRadius;
    this.kEncoderResolution = kEncoderResolution;

    this.m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public SwerveModuleState getState() {
    // if (int) motor.getHandle does not work try using the deviceId or baseId
    return new SwerveModuleState(EncoderJNI.getEncoderRate((int) speedMotor.getHandle()),
        new Rotation2d(Math.toRadians(canCoder.getPosition())));
  }

  // public void setInverted(boolean _inverted) {
  // this.sp
  // }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState,
        new Rotation2d(Math.toRadians(canCoder.getPosition())));

    // Calculate the drive output from the drive PID controller.
    // if (int) motor.getHandle does not work try using the deviceId
    // this is causing the motor to go forward and backward

    double rps = (speedMotor.getSelectedSensorVelocity() / 2048);

    SmartDashboard.putNumber("v", rps * 0.12);
    SmartDashboard.putNumber("rps", rps);

    final double driveOutput = m_drivePIDController.calculate(rps * 0.12, state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = m_turningPIDController.calculate(canCoder.getAbsolutePosition(),
        state.angle.getDegrees());

    final double turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    SmartDashboard.putNumber("driveOutput", driveOutput);
    SmartDashboard.putNumber("turnOutput [" + this.angularMotor.getDeviceID() + "] ", turnOutput);
    SmartDashboard.putNumber("SwervePod Talon[" + this.speedMotor.getDeviceID() + "] Desired Degress: ",
        desiredState.angle.getDegrees());
    SmartDashboard.putNumber("SwervePod Talon[" + this.speedMotor.getDeviceID() + "] Degress: ",
        canCoder.getAbsolutePosition());
    SmartDashboard.putNumber("SwervePod Talon[" + this.speedMotor.getDeviceID() + " Velocity: ]",
        speedMotor.getSelectedSensorVelocity());

    if (driveOutput > 0.1 || driveOutput < -0.1) {
      speedMotor.set(ControlMode.Current, driveOutput + driveFeedforward);
    }

    // angularMotor.setPosition(state.angle.getDegrees());
    angularMotor.set(ControlMode.Current, Math.floor(turnOutput + turnFeedforward));
  }
}
