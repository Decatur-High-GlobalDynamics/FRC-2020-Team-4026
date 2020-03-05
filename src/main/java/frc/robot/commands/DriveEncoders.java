/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.Constants;
import frc.robot.Utils;

public class DriveEncoders extends CommandBase {

  private DriveTrainSubsystem drive; 
  private Timer driveTime;

  private int speed;
  private int userInches;
  private int initialLeftEncoderValue;
  private int initialRightEncoderValue;
  /**
   * Creates a new DriveEncoders.
   */
  public DriveEncoders(int userInches, int speed, DriveTrainSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.userInches = userInches;
    this.speed = speed;
    this.drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTime.start();
    initialRightEncoderValue = drive.getRightEncoder();
    initialLeftEncoderValue = drive.getRightEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.setMotorPowers(speed, speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.setMotorPowers(0, 0);
    if(interrupted || driveTime.hasPeriodPassed(userInches*Constants.inchesToMeters*4))
      System.err.println("Auto interrupted!");
    if (!interrupted) {
      driveTime.stop();
      driveTime.reset();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((driveTime.hasPeriodPassed(userInches*Constants.inchesToMeters*4)) || (Utils.checkTolerance((drive.getRightEncoder()-initialRightEncoderValue)*Constants.kEncoderDistancePerPulse, userInches, Constants.driveEpsilon*Constants.kEncoderDistancePerPulse) && Utils.checkTolerance((drive.getLeftEncoder()-initialLeftEncoderValue)*Constants.kEncoderDistancePerPulse, userInches, Constants.kEncoderDistancePerPulse*Constants.driveEpsilon))){
      return true;
    } else {
      return false;
    }
  }
}
