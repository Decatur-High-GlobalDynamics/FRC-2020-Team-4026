/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turretCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class PrepareTurretCommand extends CommandBase {
  /** Creates a new PrepareTurretCommand. */
  private final TurretSubsystem turret;

  private int startingEStops;

  public PrepareTurretCommand(TurretSubsystem turret) {
    this.turret = turret;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
    startingEStops = turret.getNumEStops();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.goClockwise();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return turret.isReadyToClimb();
  }
}
