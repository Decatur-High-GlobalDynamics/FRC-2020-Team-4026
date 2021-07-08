// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*Note - this command works just like PidShootCommand with 3 changes: 1. It stops when the shooter is at the speed. 2. It doesn't stop the shooter when it ends
3. It prints to smart dashboard with a different prefix. Change 2 might be problematic
*/

package frc.robot.commands.shooterCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinUpShooterCommand extends CommandBase {
  ShooterSubsystem shooter;
  double topVelocityFraction;
  double bottomVelocityFraction;
  /** Creates a new SpinUpShooterCommand. */
  public SpinUpShooterCommand(ShooterSubsystem shooter, double topVelocityFraction, double bottomVelocityFraction) {
    this.shooter = shooter;
    this.topVelocityFraction = topVelocityFraction;
    this.bottomVelocityFraction = bottomVelocityFraction;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double topShootingVelocityFraction =
        SmartDashboard.getNumber("Commands.SpinUpShooter.topSpeedFraction", this.topVelocityFraction);
    SmartDashboard.putNumber("Commands.SpinUpShooter.topSpeedFraction", topShootingVelocityFraction);
    double bottomShootingVelocityFraction =
        SmartDashboard.getNumber(
            "Commands.SpinUpShooter.BottomSpeedFraction", this.bottomVelocityFraction);
    SmartDashboard.putNumber(
        "Commands.SpinUpShooter.BottomSpeedFraction", bottomShootingVelocityFraction);

    shooter.setMotorVelocities(topVelocityFraction, bottomVelocityFraction);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Update it every time to ensure that it keeps working - there was some weirdness this helped
    // fix
    SmartDashboard.putNumber("Commands.SpinUpShooter.topSpeedFraction", topVelocityFraction);
    SmartDashboard.putNumber("Commands.SpinUpShooter.BottomSpeedFraction", bottomVelocityFraction);
    shooter.setMotorVelocities(topVelocityFraction, bottomVelocityFraction);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooter.isShooterReady();
  }
}
