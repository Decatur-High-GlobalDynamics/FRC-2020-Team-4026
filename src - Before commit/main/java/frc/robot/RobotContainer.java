/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
<<<<<<< Updated upstream
=======


import frc.robot.commands.SimpleIntakeCommand;
import frc.robot.commands.SimpleOuttakeCommand;
import frc.robot.commands.SimpleShootCommand;
import frc.robot.commands.SimpleTurretCWCommand;
import frc.robot.commands.SimpleTurretCCWCommand;
import frc.robot.commands.TurretCWToLimit;
import frc.robot.commands.TurretCCWToLimit;
import frc.robot.commands.VerticalIndexerDownCommand;
import frc.robot.commands.VerticalIndexerUpCommand;
import frc.robot.commands.drivingCommands.TankDriveCommand;
>>>>>>> Stashed changes
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.NavigationSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrainSubsystem driveTrain = new DriveTrainSubsystem();
  private final NavigationSubsystem navigationSubsystem = new NavigationSubsystem();

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
<<<<<<< Updated upstream
=======
    //When A is held, Intake
    new JoystickButton(SecondaryJoystick, 1).whileHeld(new SimpleIntakeCommand(this.intake));
    //When X is held, Outtake
    new JoystickButton(SecondaryJoystick,2).whileHeld(new SimpleOuttakeCommand(this.intake));
    //When B is held, Indexer up
    new JoystickButton(SecondaryJoystick, 3).whileHeld(new VerticalIndexerUpCommand(this.verticalIndexer));
    //When Y is held, Indexer down
    new JoystickButton(SecondaryJoystick, 4).whileHeld(new VerticalIndexerDownCommand(this.verticalIndexer)); 
    //When left bumper  is held, Turret left
    new JoystickButton(SecondaryJoystick,5).whileHeld(new SimpleTurretCWCommand(this.turret));
    //When right bumper is held, Turret right
    new JoystickButton(SecondaryJoystick, 6).whileHeld(new SimpleTurretCCWCommand(this.turret));
    //When button 9 is pressed, turn the turret right
    new JoystickButton(SecondaryJoystick, 9).whenPressed(new TurretCWToLimit(this.turret));
    //When button 10 is pressed, turn the turret right
    new JoystickButton(SecondaryJoystick, 10).whenPressed(new TurretCCWToLimit(this.turret));
>>>>>>> Stashed changes
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // The robot will probably crash in autonomous
    return null;
  }
}
