// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivingCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.NavigationSubsystem;

//Note: Turn Command will RESET the gyro which will cause your robot to forget it's angle, if that matters
public class TurnCommand extends CommandBase {
  DriveTrainSubsystem driveTrain;
  double angle;
  double speed;
  NavigationSubsystem nav;
  /** Creates a new TurnCommand. */
  public TurnCommand(
      double angle, double speed, DriveTrainSubsystem driveTrain, NavigationSubsystem nav) {
    addRequirements(driveTrain);
    addRequirements(nav);
    nav.resetHeading();
    this.angle = angle;
    this.speed = speed;
    this.driveTrain = driveTrain;
    this.nav = nav;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    nav.resetHeading();
    if (angle < 0) {
      driveTrain.setMotorPowers(-speed, speed);
    } else {
      driveTrain.setMotorPowers(speed, -speed);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(nav.getHeading() - angle) < 2;
  }
}
