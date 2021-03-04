package frc.robot.commands.shooterCommands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class StopShootCommand extends CommandBase{
    ShooterSubsystem shooter;
    public StopShootCommand(ShooterSubsystem shooter){
        this.shooter = shooter;
        addRequirements(shooter);
    }
    @Override
    public void initialize() {}
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooter.stop();
        }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      shooter.stop();
    }
  
}
