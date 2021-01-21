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
    private final BooleanSupplier upDPad;
    private final BooleanSupplier downDPad;

    private final double curvatureDampner = 0.5;

    public GTADriveCommand(DriveTrainSubsystem driveTrain, DoubleSupplier leftStick, DoubleSupplier rightStick, BooleanSupplier rightTrigger, BooleanSupplier leftTrigger, BooleanSupplier upDPad) {
        this.driveTrain = driveTrain;
        this.leftStick = leftStick;
        this.rightStick = rightStick;
        this.rightTrigger = rightTrigger;
        this.leftTrigger = leftTrigger;
        this.upDPad = upDPad;
        this.downDPad = downDPad;

        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        double powerToSet = 0;
        if (leftTrigger.getAsBoolean() || rightTrigger.getAsBoolean()) {
            powerToSet = 0;
        }
        else if (leftTrigger.getAsBoolean()) {
            powerToSet = -1;
        }
        else if (rightTrigger.getAsBoolean()) {
            powerToSet = 1;
        }
        if (upDPad.getAsBoolean()) {
            driveTrain.setFastMode();
        } else {
            driveTrain.setSlowMode();
        }
        double turn = 0;
        if (powerToSet == 0) {
            double leftStickAmount = leftStick.getAsDouble();
            turn = leftStickAmount;
        }
        else {
            double leftStickAmount = leftStick.getAsDouble();
            turn = leftStickAmount * curvatureDampner;
        }
        driveTrain.setMotorPowers(powerToSet + turn, powerToSet - turn);
    }
}
