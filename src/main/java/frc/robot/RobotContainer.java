/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.AutoIntakeIndex;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.AutoShootTesting;
import frc.robot.commands.ConstantShootCommand;
import frc.robot.commands.HorizontalIndexerIntakeCommand;
import frc.robot.commands.HorizontalIndexerOuttakeCommand;
import frc.robot.commands.PidShootCommand;
import frc.robot.commands.SimpleClimberControlCommand;
import frc.robot.commands.SimpleIntakeCommand;
import frc.robot.commands.SimpleOuttakeCommand;
import frc.robot.commands.SimpleShootCommand;
import frc.robot.commands.SimpleTurretCCWCommand;
import frc.robot.commands.SimpleTurretCWCommand;
import frc.robot.commands.TurretToLimitCommand;
import frc.robot.commands.VerticalIndexerDownCommand;
import frc.robot.commands.VerticalIndexerUpCommand;
import frc.robot.commands.UpdateNavigationCommand;
import frc.robot.commands.drivingCommands.DriveStraightCommand;
import frc.robot.commands.drivingCommands.TankDriveCommand;
import frc.robot.commands.drivingCommands.ToggleBrakeCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NavigationSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VerticalIndexerSubsystem;
import frc.robot.subsystems.HorizontalIndexerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.CommandScheduler;



/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrainSubsystem driveTrain = new DriveTrainSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final VerticalIndexerSubsystem verticalIndexer = new VerticalIndexerSubsystem();
  private final TurretSubsystem turret = new TurretSubsystem();
  private final HorizontalIndexerSubsystem horizontalIndexer = new HorizontalIndexerSubsystem();
  private final NavigationSubsystem navigation = new NavigationSubsystem();
  private final ClimberSubsystem climber = new ClimberSubsystem();

  public static final Joystick DriveController = new Joystick(0);
  public static final Joystick SecondaryJoystick = new Joystick(1);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    //Create a button to make a BooleanSupplier off of, for the speed mode in Tank Drive. This prevents creating a new object every loop.
    final JoystickButton speedModeButton = new JoystickButton(DriveController, 7);
    //Configure driveTrain default command, which is tank drive with Primary Controller Joysticks (NUMBERED CONTROLLER). It also uses left trigger for speed mode
    driveTrain.setDefaultCommand(new TankDriveCommand(driveTrain,()->DriveController.getY(),()->DriveController.getThrottle(), ()->speedModeButton.get()));

    //Configure shooter default command, which is to spin both wheels with left joystick
    shooter.setDefaultCommand(new SimpleShootCommand(shooter,()->SecondaryJoystick.getY(),()->SecondaryJoystick.getY()));

    //Configure the default command to update our position based on encoder changes, gyro changes, and eventually vision
    navigation.setDefaultCommand(new UpdateNavigationCommand(navigation, ()->driveTrain.getLeftEncoder(), ()->driveTrain.getRightEncoder()));
    
    //Configure climber to respond to right joystick by default
    climber.setDefaultCommand(new SimpleClimberControlCommand(climber, ()->SecondaryJoystick.getThrottle()));
  


  }
  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //--------Drivetrain Button Bindings--------
    //When right trigger on main controller is held, drive straight
    new JoystickButton(DriveController, 8).whileHeld(new DriveStraightCommand(driveTrain, navigation, ()->DriveController.getY()));
    //When left bumper is pressed, toggle brake mode
    new JoystickButton(DriveController, 5).whenPressed(new ToggleBrakeCommand(driveTrain));

    //--------Intake and Indexer Button Bindings--------
<<<<<<< HEAD
    //When Y is held, Intake and Horizontal Indexer out (Synchronized)
    new JoystickButton(SecondaryJoystick, 4).whileHeld(new SimpleOuttakeCommand(this.intake).alongWith(new HorizontalIndexerOuttakeCommand(this.horizontalIndexer)));
    //When X held, Intake and Horizontal Indexer in (Synchronized)
=======
    //When X is held, Intake and Horizontal Indexer in (Synchronized)
    //new JoystickButton(SecondaryJoystick, 4).whileHeld(new SimpleOuttakeCommand(this.intake).alongWith(new HorizontalIndexerOuttakeCommand(this.horizontalIndexer)));
    //When Y held, Intake and Horizontal Indexer out (Synchronized)
>>>>>>> develop
    new JoystickButton(SecondaryJoystick, 1).whileHeld(new SimpleIntakeCommand(this.intake).alongWith(new HorizontalIndexerIntakeCommand(this.horizontalIndexer)));
    //When A is held, Intake Out
    new JoystickButton(SecondaryJoystick, 2).whileHeld(new SimpleOuttakeCommand(this.intake));
    //When B is held, Horizontal Indexer out
    new JoystickButton(SecondaryJoystick, 3).whileHeld(new HorizontalIndexerOuttakeCommand(this.horizontalIndexer));
    //When Right Trigger is held, Vertical Indexer up
    new JoystickButton(SecondaryJoystick, 8).whileHeld(new VerticalIndexerUpCommand(this.verticalIndexer));
    //When Left Trigger is held, Vertical Indexer down
    new JoystickButton(SecondaryJoystick, 7).whileHeld(new VerticalIndexerDownCommand(this.verticalIndexer));
<<<<<<< HEAD

=======
    //When button 5 is pressed (Right Bumper), shoot at constant speed
    new JoystickButton(SecondaryJoystick, 6).whileHeld(new PidShootCommand(this.shooter, 1, 1)); 
    
>>>>>>> develop
    //--------Turret Button Bindings--------
    //When left dpad is held, Turret Clockwise
    new POVButton(SecondaryJoystick, 90).whileHeld(new SimpleTurretCWCommand(this.turret));
    //When right dpad is held, Turret Counterclockwise
    new POVButton(SecondaryJoystick, 270).whileHeld(new SimpleTurretCCWCommand(this.turret));
    //When button 9 is pressed, zero the turret
    new JoystickButton(SecondaryJoystick, 9).whenPressed(new TurretToLimitCommand(this.turret));
<<<<<<< HEAD
=======

    //--------Shooting Button Bindings--------
    //When button 8 (Right Trigger) is pressed, start constant shooting
    new JoystickButton(SecondaryJoystick, 5).whileHeld(new AutoShootTesting(shooter, verticalIndexer, horizontalIndexer, intake));
    new POVButton(SecondaryJoystick, 180).whileHeld(new ConstantShootCommand(shooter));
    new JoystickButton(SecondaryJoystick, 4).whileHeld(new AutoIntakeIndex(intake, horizontalIndexer, verticalIndexer));
>>>>>>> develop
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