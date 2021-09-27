// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class Test extends CommandBase {
  CANCoder frontLeft = new CANCoder(Constants.frontLeftCANCoderId);
  CANCoder frontRight = new CANCoder(Constants.frontRightCANCoderId);
  CANCoder backLeft = new CANCoder(Constants.backLeftCANCoderId);
  CANCoder backRight = new CANCoder(Constants.backRightCANCoderId);

  /** Creates a new Reset. */
  public Test() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.frontLeft.setPosition(0);
    this.frontRight.setPosition(0);
    this.backLeft.setPosition(0);
    this.backRight.setPosition(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
