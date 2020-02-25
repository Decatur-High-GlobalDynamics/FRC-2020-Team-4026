/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class TurretToLimit extends CommandBase {
  /**
   * Creates a new TurretCWToLimit.
   */
  private final TurretSubsystem turret;
  private double calibrationTurnPower = 0.1;
  private double startTime;

  public TurretToLimit(TurretSubsystem turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turret = turret;
    addRequirements(this.turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = 1.0 * System.nanoTime() / 1e9;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.goCounterClockwise(calibrationTurnPower);
    if(((System.nanoTime() / 1e9) - startTime) > 15 || turret.isStalled()){
      this.cancel();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stop();
    if (!interrupted) {
      turret.resetEncoder();
      turret.markAsCalibrated();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return turret.getTurretLimitSwitch();
  }
}
