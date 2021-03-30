package frc.robot.commands.turretCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class PointTurretAtTargetSimple extends CommandBase {
  TurretSubsystem turret;
  VisionSubsystem vision;
  /** Creates a new PointTurretAtTargetWithAngleCommand. */
  public PointTurretAtTargetSimple(TurretSubsystem turret) {
    this.turret = turret;
    this.vision = turret.getVisionSubsystem();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (vision.isValid() && Math.abs(vision.getLastSeenTx()) > 0.1) {
      if (vision.getLastSeenTx() > 0) {
        turret.goClockwise(0.1);
      } else {
        turret.goCounterClockwise(0.1);
      }
    } else {
      turret.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
