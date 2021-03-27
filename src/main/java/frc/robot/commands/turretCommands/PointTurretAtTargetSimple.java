package frc.robot.commands.turretCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.NavigationSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.constants.TurretAimConstants;

public class PointTurretAtTargetSimple extends CommandBase {
  TurretSubsystem turret;
  VisionSubsystem vision;
  NavigationSubsystem nav;
  /** Creates a new PointTurretAtTargetWithAngleCommand. */
  public PointTurretAtTargetSimple(TurretSubsystem turret, NavigationSubsystem nav) {
    this.turret = turret;
    this.vision = turret.getVisionSubsystem();
    this.nav = nav;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (vision.isValid() && Math.abs(vision.getLastSeenTx() - getInaccuracy()) > 0.04026) {
      if (vision.getLastSeenTx() > getInaccuracy()) {
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

  public double getDistance() {
    if (vision.getLastSeenTx() == 0) {
      return TurretAimConstants.heightDifferenceForVision / Math.tan(Math.toRadians(vision.getLastSeenTy() + TurretAimConstants.limelightAngle));
    }
    return (TurretAimConstants.heightDifferenceForVision)/(Math.cos(Math.toRadians(Math.abs(nav.getHeading() + turret.getRadians() - vision.getLastSeenTx())) * Math.tan(Math.toRadians(vision.getLastSeenTy() + TurretAimConstants.limelightAngle))));
  }

  public double getInaccuracy() {
    return Math.atan(TurretAimConstants.limelightRightDist / getDistance());
  }
}