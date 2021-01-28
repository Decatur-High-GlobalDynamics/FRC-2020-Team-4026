package frc.robot.commands.drivingCommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class GTADriveCommand extends CommandBase {
  DriveTrainSubsystem driveTrain;

  private final DoubleSupplier leftStick;
  private final DoubleSupplier rightStick;
  private final BooleanSupplier rightTrigger;
  private final BooleanSupplier leftTrigger;


  public GTADriveCommand(
      DriveTrainSubsystem driveTrain,
      DoubleSupplier leftStick,
      DoubleSupplier rightStick,
      BooleanSupplier rightTrigger,
      BooleanSupplier leftTrigger) {
    this.driveTrain = driveTrain;
    this.leftStick = leftStick;
    this.rightStick = rightStick;
    this.rightTrigger = rightTrigger;
    this.leftTrigger = leftTrigger;

    addRequirements(driveTrain);
  }

  @Override
  public void execute() {
    double powerToSet = 0;
    //If both triggers are pressed, don't move as they shouldn't be
    if (leftTrigger.getAsBoolean() && rightTrigger.getAsBoolean()) {
      powerToSet = 0;
      //If just the left one is pressed, go backwards aat a speed based on the right stick
    } else if (leftTrigger.getAsBoolean()) {
      //If the right stick is all the way forward, go at max, if it's not touched, go at half, if it's pulled back, don't move
      powerToSet = -((rightStick.getAsDouble() + 1) / 2);
      //If just the right trigger is pressed, go forward
    } else if (rightTrigger.getAsBoolean()) {
      powerToSet = (rightStick.getAsDouble() + 1) / 2;
    }
    //This feeds the powers to curve drive - the power, the turn, which is leftStick's value, and whether power is super low - if power is super low we assume we're turning in place
    driveTrain.curveDrive(powerToSet, leftStick.getAsDouble(), Math.abs(powerToSet) < 0.02);
  }
}
