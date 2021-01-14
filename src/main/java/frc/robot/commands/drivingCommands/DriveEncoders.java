/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivingCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.Constants;
import frc.robot.TeamUtils;

public class DriveEncoders extends CommandBase {

  private DriveTrainSubsystem drive; 
  private Timer driveTime;

  private double speed;
  private double userMeters;
  private double initialLeftEncoderValue;
  private double initialRightEncoderValue;
  /**
   * Creates a new DriveEncoders.
   */
  public DriveEncoders(double userMeters, double speed, DriveTrainSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTime = new Timer();
    this.userMeters = userMeters;
    this.speed = speed;
    this.drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTime.reset();
    driveTime.start();
    initialRightEncoderValue = drive.getRightEncoder();
    initialLeftEncoderValue = drive.getLeftEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Subsystems.DriveTrain.timer", this.driveTime.get());
    SmartDashboard.putNumber("Subsystems.DriveTrain.leftEncoder", this.drive.getLeftEncoder());
    SmartDashboard.putNumber("Subsystems.DriveTrain.rightEncoder", this.drive.getRightEncoder());

    drive.setMotorPowers(speed, -speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.setMotorPowers(0, 0);
    if(interrupted || driveTime.hasPeriodPassed(userMeters))
      System.err.println("Auto interrupted!");
    if (!interrupted) {
      driveTime.stop();
      driveTime.reset();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  //  if (Utils.checkTolerance((Math.abs(drive.getRightEncoder()-initialRightEncoderValue))*Constants.kEncoderDistancePerPulse, Math.abs(userMeters), Constants.driveEpsilon*Constants.kEncoderDistancePerPulse) && Utils.checkTolerance((Math.abs(drive.getLeftEncoder()-initialLeftEncoderValue))*Constants.kEncoderDistancePerPulse, Math.abs(userMeters), Constants.kEncoderDistancePerPulse*Constants.driveEpsilon)){
    if((Math.abs(drive.getRightEncoder()-initialRightEncoderValue)*Constants.kDriveEncoderDistancePerPulse)>=Math.abs(userMeters) && (Math.abs(drive.getLeftEncoder()-initialLeftEncoderValue)*Constants.kDriveEncoderDistancePerPulse)>=Math.abs(userMeters) ){
    return true;
    } else {
      return false;
    }
  }
}
