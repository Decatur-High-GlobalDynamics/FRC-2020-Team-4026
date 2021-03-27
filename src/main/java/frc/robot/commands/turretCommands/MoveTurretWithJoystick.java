/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turretCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class MoveTurretWithJoystick extends CommandBase {
  /** Creates a new SimpleCCWCommand. */
  private final TurretSubsystem turret;

  private final DoubleSupplier joystick;

  public MoveTurretWithJoystick(TurretSubsystem turret, DoubleSupplier joystick) {
    this.turret = turret;
    this.joystick = joystick;
    addRequirements(turret);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Command.JoystickTurret.Joystick", joystick.getAsDouble());
    if (Math.abs(joystick.getAsDouble()) > 0.1) {
      if (joystick.getAsDouble() > 0) {
        turret.goClockwise();
      } else {
        turret.goCounterClockwise();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stop();
  }
}
