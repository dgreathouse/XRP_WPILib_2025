// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.g;

public class TurnGyro extends Command {
  double m_speed;
  double m_targetAngle_deg;
  double m_timeOut_ms;
  PIDController m_turnPID = new PIDController(g.DRIVETRAIN.TURN_KP, g.DRIVETRAIN.TURN_KI, g.DRIVETRAIN.TURN_KD);
  double m_startTime_ms = 0.0;

  /** Creates a new TurnGyro. */
  public TurnGyro(double _angle_deg, double _speed, double _timeOut_sec) {
    addRequirements(g.ROBOT.drive);
    m_targetAngle_deg = _angle_deg;
    m_speed = _speed;
    m_timeOut_ms = _timeOut_sec * 1000;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime_ms = System.currentTimeMillis();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pidValue = m_turnPID.calculate(g.ROBOT.drive.getAngle(), m_targetAngle_deg);
    pidValue = MathUtil.clamp(pidValue, -m_speed, m_speed);
    SmartDashboard.putNumber("pidValue", pidValue);
    g.ROBOT.drive.arcadeDrive(0, pidValue);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (System.currentTimeMillis() - m_startTime_ms) >= m_timeOut_ms;
    
  }
}
