// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Base;

public class PIDTester extends CommandBase {
  private Base m_base;
  private double kP = 0;
  private double kI = 0;
  private double kD = 0;
  private Timer timer = new Timer();
  private boolean isStart = true;
  private double startingTime;

  /** Creates a new PIDTester. */
  public PIDTester(Base m_base) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.m_base = m_base;

    addRequirements(m_base);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (isStart) {
      timer.start();
      this.startingTime = timer.get();
      this.isStart = false;
    }

    SmartDashboard.putNumber("timer", timer.get());

    if (this.timer.get() > 1) {
      kI += 0.10;

      this.m_base.setPID(kP, kI, kD);
      this.m_base.Drive(0.5, 0.5, 0, false);
      this.isStart = true;

      SmartDashboard.putNumber("kI", kI);
      SmartDashboard.putNumber("kD", kD);
      SmartDashboard.putNumber("kP", kP);
      this.timer.reset();
    }
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
