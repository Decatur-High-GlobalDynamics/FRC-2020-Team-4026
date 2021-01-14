/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.indexerCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HorizontalIndexerSubsystem;
import frc.robot.subsystems.VerticalIndexerSubsystem;

public class TransferBall extends CommandBase {
  /**
   * Creates a new TransferBall.
   */
  private final VerticalIndexerSubsystem verticalIndexer;
  private final HorizontalIndexerSubsystem horizontalIndexer;

  private Double ticksWhenBottomIsUnpressed = null;

  public TransferBall(VerticalIndexerSubsystem verticalIndexer, HorizontalIndexerSubsystem horizontalIndexer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.verticalIndexer = verticalIndexer;
    this.horizontalIndexer = horizontalIndexer;
    addRequirements(verticalIndexer, horizontalIndexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Refuse to start the vertical indexer if top switch is pressed
    if (!verticalIndexer.topSwitchIsPressed()){
      verticalIndexer.up();
      horizontalIndexer.intake();
      ticksWhenBottomIsUnpressed = null;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (ticksWhenBottomIsUnpressed == null && !verticalIndexer.bottomSwitchIsPressed()) {
      ticksWhenBottomIsUnpressed = verticalIndexer.getPosition();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    verticalIndexer.stop();
    horizontalIndexer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (verticalIndexer.topSwitchIsPressed()){
      return true;
    }
    if (ticksWhenBottomIsUnpressed==null){
      return false;
    }
    return Math.abs(verticalIndexer.getPosition() - ticksWhenBottomIsUnpressed) >= verticalIndexer.ticksUntilTransfered;
  }
}
