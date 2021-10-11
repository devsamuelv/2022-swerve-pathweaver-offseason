// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;

public class ShooterStartUp extends CommandBase {

  private final Shooter m_subsystem;
  private final Hopper m_hopper;
  private final Intake m_intake;

  public ShooterStartUp(Shooter shooter, Intake intake, Hopper hopper) {
    m_subsystem = shooter;
    m_intake = intake;
    m_hopper = hopper;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_subsystem.ReachedVelocity == true) {
      m_intake.setSpeed(1);
      m_hopper.setSpeed(1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setSpeed(0);
    m_hopper.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
