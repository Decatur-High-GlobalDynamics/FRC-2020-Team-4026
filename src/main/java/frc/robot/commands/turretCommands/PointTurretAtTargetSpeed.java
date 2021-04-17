// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turretCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class PointTurretAtTargetSpeed extends CommandBase {
  TurretSubsystem turret;
  VisionSubsystem vision;
  final double fastSpeed = 500;
  final double slowSpeed = 250;
  /** Creates a new PointTurretAtTargetWithAngleCommand. */
  public PointTurretAtTargetSpeed(TurretSubsystem turret) {
    this.turret = turret;
    this.vision = turret.getVisionSubsystem();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (vision.isValid()) {
      double angleError = vision.getLastSeenTx();
      if (angleError > 0.1) {
        if (Math.abs(angleError) > 10) {
          turret.turnAtVelocity(fastSpeed);
        } else {
          turret.turnAtVelocity(slowSpeed);
        }
      }
      else if (angleError < -0.1) {
        if (Math.abs(angleError) > 10) {
          turret.turnAtVelocity(fastSpeed);
        } else {
          turret.turnAtVelocity(slowSpeed);
        }
      }
    } else {
      turret.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
