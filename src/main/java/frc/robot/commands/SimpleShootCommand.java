/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class SimpleShootCommand extends CommandBase {
  
  ShooterSubsystem shooter;
  DoubleSupplier topThrottle;
  DoubleSupplier bottomThrottle;
  /**
   * Creates a new SimpleShootCommand.
   */
  public SimpleShootCommand(ShooterSubsystem shooter, DoubleSupplier top, DoubleSupplier bottom) {
    this.shooter = shooter;
    this.topThrottle = top;
    this.bottomThrottle = bottom;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setBottomMotor(this.bottomThrottle.getAsDouble());
    shooter.setTopMotor(this.topThrottle.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
