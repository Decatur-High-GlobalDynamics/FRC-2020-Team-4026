/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivingCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.NavigationSubsystem;

public class DriveStraightCommand extends CommandBase {
  DriveTrainSubsystem driveTrain;
  NavigationSubsystem nav;
  DoubleSupplier joystick;

  // This is the turn offset p value for pid
  double pTurn = 1 / 90;

  double desiredHeading;

  /** Creates a new DriveStraightCommand. */
  public DriveStraightCommand(
      DriveTrainSubsystem driveTrain, NavigationSubsystem nav, DoubleSupplier joystick) {
    addRequirements(driveTrain);
    this.driveTrain = driveTrain;
    this.nav = nav;
    this.joystick = joystick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    desiredHeading = nav.getAccumulatedHeading();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double desiredSpeed = joystick.getAsDouble();
    // This is the offset for motors for turning
    double turnOffset = 0;

    driveTrain.setMotorPowers(-(desiredSpeed) + turnOffset, desiredSpeed + turnOffset);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
