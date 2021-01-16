/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.TeamUtils;
import frc.robot.subsystems.TurretSubsystem;

public class PointTurretAtTargetCommand extends CommandBase {
  TurretSubsystem turret;
  Timer timeoutForNoVision;

  /** Creates a new PointTurretAtTarget Command. */
  public PointTurretAtTargetCommand(TurretSubsystem turret) {
    this.turret = turret;
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angleToTarget;
    // Get the value of vision angle from limelight
    angleToTarget = turret.getVisionXAngle();
    // This is sent if the target isn't seen
    if (angleToTarget == 4026) {
      turret.stop();
    } else {
      // If angle is more than 0, it is on our right so we go clockwise in a proportion of the angle
      // we are off
      if (TeamUtils.checkTolerance(angleToTarget, 0, .5)) {
        turret.stop();
      } else if (angleToTarget > 0) {
        turret.goClockwise();
        // If angle is less than 0, it is on our left so we go counter clockwise in a proportion of
        // our angle
      } else if (angleToTarget < 0) {
        turret.goCounterClockwise();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
