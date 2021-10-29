/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climberCommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class SimpleClimberControlCommand extends CommandBase {
  /** Creates a new SimpleClimberControlCommand. */
  private ClimberSubsystem climber;
  private TurretSubsystem turret;

  private DoubleSupplier leftPower;
  private DoubleSupplier rightPower;

  private BooleanSupplier dpadUp;

  boolean overridden = false;

  public SimpleClimberControlCommand(
      ClimberSubsystem climber, DoubleSupplier leftPower, DoubleSupplier rightPower, TurretSubsystem turret, BooleanSupplier dpadUp) {
    this.climber = climber;
    this.leftPower = leftPower;
    this.rightPower = rightPower;
    this.turret = turret;
    this.dpadUp = dpadUp;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (dpadUp.getAsBoolean()) {
      overridden = true;
    }
    if (turret.isReadyToClimb() || overridden) {
      this.climber.setLeftClimber(this.leftPower.getAsDouble());
      this.climber.setRightClimber(this.rightPower.getAsDouble());
    } else {
      this.climber.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
