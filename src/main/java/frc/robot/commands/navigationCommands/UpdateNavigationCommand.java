package frc.robot.commands.navigationCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.NavigationSubsystem;

public class UpdateNavigationCommand extends CommandBase {
    DoubleSupplier leftEncoder;
    DoubleSupplier rightEncoder;
    NavigationSubsystem navSystem;

    public UpdateNavigationCommand(NavigationSubsystem nav, DoubleSupplier leftEncode, DoubleSupplier rightEncode) {
        navSystem = nav;
        leftEncoder = leftEncode;
        rightEncoder = rightEncode;

        addRequirements(nav);
    }
    //This just updates position. Vision and stuff should be added here (i.e. this can query vision and if it exists do that after it updates pose normally)
    @Override
    public void execute() {
        //Update the pose based on encoders
        navSystem.updatePoseNormally(leftEncoder.getAsDouble(), rightEncoder.getAsDouble());
    }
}