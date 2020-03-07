/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utils;
import frc.robot.subsystems.TurretSubsystem;

public class PointTurretAtTargetCommand extends CommandBase {
  TurretSubsystem turret;
  Timer timeoutForNoVision;

  final double kProportionConstant = 0.005;

  /**
   * Creates a new PointTurretAtTarget Command.
   */
  public PointTurretAtTargetCommand(TurretSubsystem turret) {
    this.turret = turret;
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timeoutForNoVision.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angleToTarget;
    try{
      //Get the value of field xAngle in table angles from the network table
      angleToTarget = (double)Utils.getFromNetworkTable("angles", "xAngle");
    }
    catch(NullPointerException e){
      System.err.println("Angle not found in NetworkTables. Is the Pi Connected?");
      //4026 is the condition if vision isn't working or can't see the target
      angleToTarget = 4026;
    }
    //This is sent if the target isn't seen
    if (angleToTarget == 4026) {
      turret.stop();
    } else {
      //If angle is more than 0, it is on our right so we go clockwise in a proportion of the angle we are off
      if (angleToTarget > 0) {
        turret.goClockwise(kProportionConstant * Math.abs(angleToTarget));
      //If angle is less than 0, it is on our left so we go counter clockwise in a proportion of our angle
      } else if (angleToTarget < 0) {
        turret.goCounterClockwise(kProportionConstant * Math.abs(angleToTarget));
      //
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
