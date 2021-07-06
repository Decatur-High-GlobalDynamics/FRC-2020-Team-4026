// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooterCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShootPower extends CommandBase {
  ShooterSubsystem shooter;
  /** Creates a new AutoShootPower. */
  public AutoShootPower(ShooterSubsystem shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double velocityFraction = getPowerFromDistance(shooter.getKnotDistance());
    SmartDashboard.putNumber("Commands.PidShooter.velocityFraction", velocityFraction);
    shooter.setMotorVelocities(velocityFraction, velocityFraction);
  }

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

  private double getPowerFromDistance(double distance) {
    return 1;
  }
}
