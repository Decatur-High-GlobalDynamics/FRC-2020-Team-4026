package frc.robot.commands;

import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.NavigationSubsystem;

public class UpdateNavigationCommand extends CommandBase {
    IntSupplier leftEncoder;
    IntSupplier rightEncoder;
    NavigationSubsystem navSystem;

    public UpdateNavigationCommand(NavigationSubsystem nav, IntSupplier leftEncode, IntSupplier rightEncode) {
        navSystem = nav;
        leftEncoder = leftEncode;
        rightEncoder = rightEncode;

        addRequirements(nav);
    }
    //This just updates position. Vision and stuff should be added here (i.e. this can query vision and if it exists do that after it updates pose normally)
    @Override
    public void execute() {
        //Update the pose based on encoders
        navSystem.updatePoseNormally(leftEncoder.getAsInt(), rightEncoder.getAsInt());
    }
}