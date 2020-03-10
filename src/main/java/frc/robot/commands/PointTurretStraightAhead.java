/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.TeamUtils;
import frc.robot.subsystems.TurretSubsystem;

public class PointTurretStraightAhead extends CommandBase {
  TurretSubsystem turret;
  /**
   * Creates a new PointTurretStraightAhead.
   */
  public PointTurretStraightAhead(TurretSubsystem turret) {
    this.turret = turret;
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.startRotatingToEncoderPosition(-2914);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return TeamUtils.checkTolerance(Math.PI/2, turret.getRadians(), 1/(36*Math.PI)) || !turret.checkCalibration();
  }
}
