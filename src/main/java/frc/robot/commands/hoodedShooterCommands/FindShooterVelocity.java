/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hoodedShooterCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HoodedShooterSubsystem;

public class FindShooterVelocity extends CommandBase {
  /** Creates a new FindTurretVelocity. */
  private final double percentTarget;

  private int currVel = 0;
  private final HoodedShooterSubsystem shooter;
  private boolean done = false;

  public FindShooterVelocity(HoodedShooterSubsystem shooter, double percentTarget) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.percentTarget = percentTarget;
    this.shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooter.getShooterPower() < percentTarget) {
      currVel += 5;
      shooter.setMotor(currVel);
    } else {
      SmartDashboard.putNumber("80% power velocity", shooter.getShooterPower());
      done = true;
      shooter.setMotor(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
