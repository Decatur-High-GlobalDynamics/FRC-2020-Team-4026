/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;


public class StopTurretCommand extends CommandBase {
  /**
   * Creates a new StopTurretCommand.
   */
  private final TurretSubsystem turret;
  public StopTurretCommand(TurretSubsystem turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turret = turret;
    addRequirements(this.turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()  {
    turret.stop();
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
