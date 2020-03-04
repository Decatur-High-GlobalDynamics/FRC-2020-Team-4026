/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.NetworkIOSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class PointTurretAtTargetCommand extends CommandBase {
  TurretSubsystem turret;
  NetworkIOSubsystem network;
  Timer cantSeeTimeout;

  /**
   * Creates a new PointTurretAtTarget Command.
   */
  public PointTurretAtTargetCommand(TurretSubsystem turret, NetworkIOSubsystem network) {
    this.turret = turret;
    this.network = network;
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cantSeeTimeout.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xAngle;
    try{
     xAngle = (double)network.get("angles", "xAngle");
    }
    catch(NullPointerException e){
      System.err.println("Angle not found in NetworkTables. Is the Pi Connected?");
      xAngle = 4026;
    }
    //This is sent if the target isn't seen
    if (xAngle == 4026) {
      if (cantSeeTimeout.get() > 2) {
        turret.stop();
      }
    } else {
      cantSeeTimeout.reset();
      double xAngleInRads = xAngle * (Math.PI / 180);
      double targetRads = turret.getRadians() + xAngleInRads;
      if (turret.isRadsAllowed(targetRads)) {
        turret.startRotatingToPosition(targetRads);
      } else {
        turret.stop();
      }
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
