/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hoodedShooterCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HoodedShooterNoPID;

public class PowerCommandShooter extends CommandBase {
  HoodedShooterNoPID shooter;
  double power;
  /** Creates a new SimpleShootCommand. */
  public PowerCommandShooter(HoodedShooterNoPID shooter, double power) {
    this.shooter = shooter;
    this.power = power;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double shootingPower =
        SmartDashboard.getNumber("Commands.PowerShooter.power", this.power);
    SmartDashboard.putNumber("Commands.PowerShooter.power", shootingPower);

    shooter.setShooterPower(power);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Update it every time to ensure that it keeps working - there was some weirdness this helped
    // fix
    SmartDashboard.putNumber("Commands.PowerShooter.power", power);
    shooter.setShooterPower(power);
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
