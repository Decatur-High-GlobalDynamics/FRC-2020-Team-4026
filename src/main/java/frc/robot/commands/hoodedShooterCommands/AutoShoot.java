/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hoodedShooterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.indexerCommands.HorizontalIndexerIntakeCommand;
import frc.robot.commands.indexerCommands.VerticalIndexerUpCommand;
import frc.robot.commands.turretCommands.PointTurretAtTargetWithAngleCommand;
import frc.robot.subsystems.HoodedShooterSubsystem;
import frc.robot.subsystems.HorizontalIndexerSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VerticalIndexerSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoShoot extends CommandBase {
  /** Creates a new AutoShoot. */
  private final HoodedShooterSubsystem shooter;
  private final VisionSubsystem vision;
  private final PointTurretAtTargetWithAngleCommand aimTurret;
  private final HorizontalIndexerIntakeCommand horizontalIntake;
  private final VerticalIndexerUpCommand verticalUp;

  public AutoShoot(
      HoodedShooterSubsystem shooter, VerticalIndexerSubsystem verticalIndexer, HorizontalIndexerSubsystem horizontalIndexer, VisionSubsystem vision, TurretSubsystem turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.vision = vision;
    this.horizontalIntake = new HorizontalIndexerIntakeCommand(horizontalIndexer);
    this.verticalUp = new VerticalIndexerUpCommand(verticalIndexer);
    this.aimTurret = new PointTurretAtTargetWithAngleCommand(turret);
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    aimTurret.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (vision.isValid()) {
      shooter.setShooterVelFraction(getTargetSpeed(vision.getLastSeenTy()));
      if (shooter.isShooterReady() && Math.abs(vision.getLastSeenTx()) < 1) {
        verticalUp.schedule();
        horizontalIntake.schedule();
      } else {
        verticalUp.cancel();
        horizontalIntake.cancel();
      }
    } else {
      shooter.stop();
      verticalUp.cancel();
      horizontalIntake.cancel();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    aimTurret.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private double getTargetSpeed(double currentAngle) {
    return 0.5;
  }
}
