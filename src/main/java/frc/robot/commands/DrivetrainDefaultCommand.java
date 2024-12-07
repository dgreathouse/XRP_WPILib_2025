// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.lib.g;
import edu.wpi.first.wpilibj2.command.Command;

public class DrivetrainDefaultCommand extends Command {
double cnt = 0;

  /**
   * Creates a new ArcadeDrive. This command will drive your robot according to the speed supplier
   * lambdas. This command does not terminate.
   *
   * @param xaxisSpeedSupplier Lambda supplier of forward/backward speed
   * @param zaxisRotateSupplier Lambda supplier of rotational speed
   */
  public DrivetrainDefaultCommand() {
    addRequirements(g.ROBOT.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = g.OI.driverController.getRawAxis(1);
    double y = g.OI.driverController.getRawAxis(0);
    g.ROBOT.drive.arcadeDrive(x,y);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(g.ROBOT.rangeFinder.getDistanceInches() < 10){
      g.DRIVETRAIN.speedLimiter = 0.5;
    }
    return false;
  }
}
