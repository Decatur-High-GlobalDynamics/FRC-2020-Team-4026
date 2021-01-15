/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivingCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class TankDriveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  DriveTrainSubsystem driveTrain;

  private final DoubleSupplier leftStick;
  private final DoubleSupplier rightStick;

  /**
   * Creates a new TankDriveCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TankDriveCommand(DriveTrainSubsystem driveTrain, DoubleSupplier left, DoubleSupplier right) {
    this.driveTrain = driveTrain;
    this.leftStick = left;
    this.rightStick = right;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.setMotorPowers(-this.leftStick.getAsDouble(), this.rightStick.getAsDouble());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
