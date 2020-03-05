/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class TurretToTarget extends CommandBase {
  /**
   * Creates a new TurretToTarget.
   */
  private final TurretSubsystem turret;
  private final SimpleTurretCCWCommand CCWCommand;
  private final SimpleTurretCWCommand CWCommand;

  public TurretToTarget(TurretSubsystem turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turret = turret;
    CWCommand = new SimpleTurretCWCommand(turret);
    CCWCommand = new SimpleTurretCCWCommand(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    if (turret.getVisionXAngle() == 4026.0){
      CWCommand.cancel();
      CCWCommand.cancel();
    } else if (turret.getVisionXAngle() < 0){
      CWCommand.cancel();
      CCWCommand.schedule();
    } else if (turret.getVisionXAngle() > 0){
      CCWCommand.cancel();
      CWCommand.schedule();
    } else {
      CWCommand.cancel();
      CCWCommand.cancel();
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
    return Math.abs(turret.getVisionXAngle()) <= 10;
  }
}
