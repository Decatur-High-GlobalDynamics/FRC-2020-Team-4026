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
import frc.robot.subsystems.VisionSubsystem;

/**
 * TODO in the future: make this better, I want to have real pathing once pathfinding is a thing,
 * instead of turning in place then going forwards
 */
public class HunterKillerCommand extends CommandBase {
  DriveTrainSubsystem driveTrain;
  NavigationSubsystem nav;
  DoubleSupplier joystick;
  VisionSubsystem vision;

  // This is the turn offset p value for pid
  double pTurn = 1 / 90;

  double desiredHeading;

  // number of calls to the execute function where the robot's appeared to be aligned to the ball
  int alignCounter = 0;

  /**
   * Creates a new HunterKillerCommand. Assumes that the camera is fixed to the drivetrain, oriented
   * forward
   */
  public HunterKillerCommand(
      DriveTrainSubsystem driveTrain,
      NavigationSubsystem nav,
      DoubleSupplier joystick,
      VisionSubsystem vision) {
    addRequirements(driveTrain);
    this.driveTrain = driveTrain;
    this.nav = nav;
    this.joystick = joystick;
    this.vision = vision;
    this.alignCounter = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    desiredHeading = nav.getAccumulatedHeading();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double desiredSpeed = joystick.getAsDouble();
    double desiredSpeed = 0.5;
    // This is the offset for motors for turning
    double turnOffset = 0;

    // Variable to represent the ball's x coordinate that means the robot is aligned with it
    double angleMid = 0;

    // allowed range around the angleMid for the robot to be aligned with the ball
    double angleOff = 0.1;

    if (vision.getBallSeen()) {

      double ballX = vision.getBallX();

      if (this.alignCounter < 2) {
        if (Math.abs(ballX - angleMid) < angleOff) {

          // drive straight if the ball is in front of the bot
          driveTrain.setMotorPowers(-(desiredSpeed) + turnOffset, desiredSpeed + turnOffset);
          this.alignCounter++;

        } else {
          if (ballX > 0) {

            // turn right if the ball is right of the robot
            driveTrain.setMotorPowers(-(desiredSpeed) + turnOffset, -(desiredSpeed + turnOffset));

          } else {

            // turn left if the ball is left of the robot
            driveTrain.setMotorPowers(
                -(-(desiredSpeed) + turnOffset), -(desiredSpeed + turnOffset));
          }
        }
      }
    }
    // if we have 3 or more frames where the robot was aligned with the ball
    if (this.alignCounter > 2) {

      // drive straight if the ball already aligned with the ball
      driveTrain.setMotorPowers(-(desiredSpeed) + turnOffset, desiredSpeed + turnOffset);
    }
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
