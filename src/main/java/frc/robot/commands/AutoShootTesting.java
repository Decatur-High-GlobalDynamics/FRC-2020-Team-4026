/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HorizontalIndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VerticalIndexerSubsystem;

public class AutoShootTesting extends CommandBase {
  /**
   * Creates a new AutoShoot.
   */
  private final ShooterSubsystem shooter;
  private final VerticalIndexerSubsystem verticalIndexer;
  private final HorizontalIndexerSubsystem horizontalIndexer;
  private final IntakeSubsystem intake;
  private int targetSpeedTop;
  private int targetSpeedBot;

  public AutoShootTesting(ShooterSubsystem shooter, VerticalIndexerSubsystem verticalIndexer, HorizontalIndexerSubsystem horizontalIndexer, IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.verticalIndexer = verticalIndexer;
    this.horizontalIndexer = horizontalIndexer;
    
    this.intake = intake;
    addRequirements(shooter);
    addRequirements(verticalIndexer);
    addRequirements(horizontalIndexer);
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //In the future, get speeds from the lookup table based on vision
    //Also, potentially rotate turret
    targetSpeedBot = shooter.getTargetSpeedBot();
    targetSpeedTop = (int) (targetSpeedBot * (2.5 / 6.5));
    shooter.setShooterVelBot(targetSpeedBot);
    shooter.setShooterVelTop(targetSpeedTop);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooter.isShooterReady()){
      horizontalIndexer.intake();
      verticalIndexer.up();
      intake.inTake();
    } else {
      horizontalIndexer.stop();
      verticalIndexer.stop();
    }
    SmartDashboard.putNumber("Commands.AutoShootTestingCommand.targetSpeedTop", targetSpeedTop);
    SmartDashboard.putNumber("Commands.AutoShootTestingCommand.targetSpeedBot", targetSpeedBot);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setShooterVelTop(0);
    shooter.setShooterVelBot(0);
    horizontalIndexer.stop();
    verticalIndexer.stop();
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
