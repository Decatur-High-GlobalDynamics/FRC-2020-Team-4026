/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HorizontalIndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VerticalIndexerSubsystem;

public class AutoShoot extends CommandBase {
  /**
   * Creates a new AutoShoot.
   */
  private final ShooterSubsystem shooterSubsystem;
  private final VerticalIndexerSubsystem verticalIndexer;
  private final HorizontalIndexerSubsystem horizontalIndexer;
  private final int targetSpeedTop;
  private final int targetSpeedBot;

  public AutoShoot(ShooterSubsystem shooterSubsystem, VerticalIndexerSubsystem verticalIndexer, HorizontalIndexerSubsystem horizontalIndexer, int targetSpeedTop, int targetSpeedBot) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSubsystem = shooterSubsystem;
    this.verticalIndexer = verticalIndexer;
    this.horizontalIndexer = horizontalIndexer;
    this.targetSpeedTop = targetSpeedTop;
    this.targetSpeedBot = targetSpeedBot;
    addRequirements(shooterSubsystem);
    addRequirements(verticalIndexer);
    addRequirements(horizontalIndexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //In the future, get speeds from the lookup table based on vision
    //Also, potentially rotate turret
    shooterSubsystem.setShooterVelBot(targetSpeedBot);
    shooterSubsystem.setShooterVelTop(targetSpeedTop);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooterSubsystem.getShooterSpeedBot() >= targetSpeedBot && shooterSubsystem.getShooterSpeedTop() >= targetSpeedTop){
      horizontalIndexer.intake();
      verticalIndexer.up();
    } else {
      horizontalIndexer.stop();
      verticalIndexer.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setShooterVelTop(0);
    shooterSubsystem.setShooterVelBot(0);
    horizontalIndexer.stop();
    verticalIndexer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
