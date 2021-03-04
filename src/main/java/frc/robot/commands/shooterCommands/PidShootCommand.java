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

public class PidShootCommand extends CommandBase {
  ShooterSubsystem shooter;
  double topVelocityFraction;
  double bottomVelocityFraction;
  /** Creates a new SimpleShootCommand. */
  public PidShootCommand(
      ShooterSubsystem shooter, double topVelocityFraction, double bottomVelocityFraction) {
    this.shooter = shooter;
    this.topVelocityFraction = topVelocityFraction;
    this.bottomVelocityFraction = bottomVelocityFraction;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double topShootingVelocityFraction =
        SmartDashboard.getNumber("Commands.PidShooter.topSpeedFraction", this.topVelocityFraction);
    SmartDashboard.putNumber("Commands.PidShooter.topSpeedFraction", topShootingVelocityFraction);
    double bottomShootingVelocityFraction =
        SmartDashboard.getNumber(
            "Commands.PidShooter.BottomSpeedFraction", this.bottomVelocityFraction);
    SmartDashboard.putNumber(
        "Commands.PidShooter.BottomSpeedFraction", bottomShootingVelocityFraction);

    shooter.setMotorVelocities(topShootingVelocityFraction, bottomShootingVelocityFraction);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
