/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooterCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class FindShooterVelocity extends CommandBase {
  /** Creates a new FindTurretVelocity. */
  private final double percentTarget;

  private int currVelBot = 0;
  private int currVelTop = 0;
  private final ShooterSubsystem shooter;
  private boolean botDone = false;
  private boolean topDone = false;

  public FindShooterVelocity(ShooterSubsystem shooter, double percentTarget) {
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
    if (shooter.getShooterPowerBot() < percentTarget) {
      currVelBot += 5;
      shooter.setShooterVelBot(currVelBot);
    } else {
      SmartDashboard.putNumber("80% power bot velocity", shooter.getShooterSpeedBot());
      botDone = true;
      shooter.setTopMotor(0);
    }

    if (shooter.getShooterPowerTop() < percentTarget) {
      currVelTop += 5;
      shooter.setBottomMotor(currVelTop);
    } else {
      SmartDashboard.putNumber("80% power top velocity", shooter.getShooterPowerTop());
      topDone = true;
      shooter.setTopMotor(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return topDone && botDone;
  }
}
