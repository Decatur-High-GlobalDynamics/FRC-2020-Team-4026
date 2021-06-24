/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HoodedShooterSubsystem;
import frc.robot.subsystems.HorizontalIndexerSubsystem;
import frc.robot.subsystems.VerticalIndexerSubsystem;

public class AutoHoodedShootWithHorizontal extends CommandBase {
  /** Creates a new AutoShoot. */
  private final HoodedShooterSubsystem shooter;

  private final VerticalIndexerSubsystem verticalIndexer;
  private final HorizontalIndexerSubsystem horizontalIndexer;
  private final int targetSpeed;

  public AutoHoodedShootWithHorizontal(
      HoodedShooterSubsystem shooter,
      VerticalIndexerSubsystem verticalIndexer,
      HorizontalIndexerSubsystem horizontalIndexer,
      int targetSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.verticalIndexer = verticalIndexer;
    this.horizontalIndexer = horizontalIndexer;
    this.targetSpeed = targetSpeed;
    addRequirements(shooter);
    addRequirements(verticalIndexer);
    addRequirements(horizontalIndexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // In the future, get speeds from the lookup table based on vision
    // Also, potentially rotate turret
    shooter.setShooterVelLeft(targetSpeed);
    shooter.setShooterVelRight(targetSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooter.isShooterReady()) {
      verticalIndexer.up();
      horizontalIndexer.intake();
    } else {
      verticalIndexer.stop();
      horizontalIndexer.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setShooterVelLeft(0);
    shooter.setShooterVelRight(0);
    verticalIndexer.stop();
    horizontalIndexer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
