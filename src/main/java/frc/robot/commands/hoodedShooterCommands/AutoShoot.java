/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hoodedShooterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HoodedShooterSubsystem;
import frc.robot.subsystems.VerticalIndexerSubsystem;

public class AutoShoot extends CommandBase {
  /** Creates a new AutoShoot. */
  private final HoodedShooterSubsystem shooter;

  private final VerticalIndexerSubsystem verticalIndexer;
  private final int targetSpeed;

  public AutoShoot(
      HoodedShooterSubsystem shooter, VerticalIndexerSubsystem verticalIndexer, int targetSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.verticalIndexer = verticalIndexer;
    this.targetSpeed = targetSpeed;
    addRequirements(shooter);
    addRequirements(verticalIndexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // In the future, get speeds from the lookup table based on vision
    // Also, potentially rotate turret
    shooter.setShooterVel(targetSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooter.isShooterReady()) {
      verticalIndexer.up();
    } else {
      verticalIndexer.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    verticalIndexer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
