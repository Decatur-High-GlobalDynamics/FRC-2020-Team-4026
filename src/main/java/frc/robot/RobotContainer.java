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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.AutoIntakeIndex;
import frc.robot.commands.AutoShootWithHorizontal;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.shooterCommands.ConstantShootCommand;
import frc.robot.commands.indexerCommands.HorizontalIndexerIntakeCommand;
import frc.robot.commands.indexerCommands.HorizontalIndexerOuttakeCommand;
import frc.robot.commands.shooterCommands.PidShootCommand;
import frc.robot.commands.climberCommands.SimpleClimberControlCommand;
import frc.robot.commands.intakeCommands.SimpleIntakeCommand;
import frc.robot.commands.intakeCommands.SimpleOuttakeCommand;
import frc.robot.commands.turretCommands.SimpleTurretCCWCommand;
import frc.robot.commands.turretCommands.SimpleTurretCWCommand;
import frc.robot.commands.turretCommands.PointTurretAtTargetWithAngleCommand;
import frc.robot.commands.turretCommands.PrepareTurretCommand;
import frc.robot.commands.drivingCommands.DriveEncoders;
import frc.robot.commands.turretCommands.TurretToLimitCommand;
import frc.robot.commands.indexerCommands.VerticalIndexerDownCommand;
import frc.robot.commands.indexerCommands.VerticalIndexerUpCommand;
import frc.robot.commands.navigationCommands.UpdateNavigationCommand;
import frc.robot.commands.drivingCommands.DisableRampingCommand;
import frc.robot.commands.drivingCommands.DriveStraightCommand;
import frc.robot.commands.drivingCommands.EnableBrakeModeCommand;
import frc.robot.commands.drivingCommands.SetSpeedMode;
import frc.robot.commands.shooterCommands.MaxPowerShootCommand;
import frc.robot.commands.drivingCommands.TankDriveCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NavigationSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VerticalIndexerSubsystem;
import frc.robot.subsystems.HorizontalIndexerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;


//TODO: Bind to buttons
import frc.robot.commands.AutoShoot;

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

  public static final Joystick driveController = new Joystick(0);
  public static final Joystick secondaryJoystick = new Joystick(1);

  enum PossibleAutos {
    IN_FRONT_OF_TARGET_MAX_POWER,
    IN_FRONT_OF_TARGET_MAX_POWER_THEN_BACK,
  }


  SendableChooser<PossibleAutos> autoChoice = new SendableChooser<PossibleAutos>();

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureDriveController();
    configureSecondaryController();

    //Add options for auto choice
    addAutoChoicesToGui();

    //Configure the default command to update our position based on encoder changes, gyro changes, and eventually vision
    navigation.setDefaultCommand(new UpdateNavigationCommand(navigation, ()->driveTrain.getLeftEncoder(), ()->driveTrain.getRightEncoder()));
  }
  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void configureDriveController() {
    Button rightTrigger = new JoystickButton(driveController, LogitechControllerButtons.triggerRight);
    Button leftTrigger = new JoystickButton(driveController, LogitechControllerButtons.triggerLeft);
    Button leftBumper = new JoystickButton(driveController, LogitechControllerButtons.bumperLeft);
    Button rightBumper = new JoystickButton(driveController, LogitechControllerButtons.bumperRight);
    

    //Configure driveTrain default command, which is tank drive with Primary Controller Joysticks (NUMBERED CONTROLLER). It also uses left trigger for speed mode
    driveTrain.setDefaultCommand(new TankDriveCommand(driveTrain,()->driveController.getY(),()->driveController.getThrottle()));

    //--------Drivetrain Button Bindings--------
    //When right trigger on main controller is held, drive straight
    rightTrigger.whileHeld(new DriveStraightCommand(driveTrain, navigation, ()->driveController.getY()));
    //When left trigger is held, set speed mode
    leftTrigger.whileHeld(new SetSpeedMode(driveTrain));
    //When left bumper held, enable brake mode
    leftBumper.whileHeld(new EnableBrakeModeCommand(driveTrain));
    //When right bumper held, disable ramping
    rightBumper.whileHeld(new DisableRampingCommand(driveTrain));
  }

  private void configureSecondaryController() {
    Button x = new JoystickButton(secondaryJoystick, LogitechControllerButtons.x);
    Button a = new JoystickButton(secondaryJoystick, LogitechControllerButtons.a);
    Button b = new JoystickButton(secondaryJoystick, LogitechControllerButtons.b);
    Button y = new JoystickButton(secondaryJoystick, LogitechControllerButtons.y);
    Button rightBumper = new JoystickButton(secondaryJoystick, LogitechControllerButtons.bumperRight);
    Button leftBumper = new JoystickButton(secondaryJoystick,LogitechControllerButtons.bumperLeft);
    Button leftTrigger = new JoystickButton(secondaryJoystick, LogitechControllerButtons.triggerLeft);
    Button rightTrigger = new JoystickButton(secondaryJoystick, LogitechControllerButtons.triggerRight);
    Button home = new JoystickButton(secondaryJoystick, LogitechControllerButtons.home);
    Button start = new JoystickButton(secondaryJoystick, LogitechControllerButtons.start);

    Button dPadUp = new POVButton(secondaryJoystick, LogitechControllerButtons.up);
    Button dPadRight = new POVButton(secondaryJoystick, LogitechControllerButtons.right);
    Button dPadDown = new POVButton(secondaryJoystick, LogitechControllerButtons.down);
    Button dPadLeft = new POVButton(secondaryJoystick, LogitechControllerButtons.left);

    //Configure climber to respond to both joysticks by default
    climber.setDefaultCommand(new SimpleClimberControlCommand(climber, ()->secondaryJoystick.getY(),()->secondaryJoystick.getThrottle()));

    //--------Intake and Indexer Button Bindings--------
    //When Y is held, Intake and Horizontal Indexer out (Synchronized)
    y.whileHeld(new AutoIntakeIndex(intake, horizontalIndexer, verticalIndexer));
    //When X held, Intake and Horizontal Indexer in (Synchronized)
    x.whileHeld(new SimpleIntakeCommand(this.intake).alongWith(new HorizontalIndexerIntakeCommand(this.horizontalIndexer)));
    //When A is held, Intake Out
    a.whileHeld(new SimpleOuttakeCommand(this.intake));
    //When B is held, Horizontal Indexer out
    b.whileHeld(new HorizontalIndexerOuttakeCommand(this.horizontalIndexer));
    //When Right Trigger is held, Vertical Indexer up
    rightTrigger.whileHeld(new VerticalIndexerUpCommand(this.verticalIndexer));
    //When Left Trigger is held, Vertical Indexer down
    leftTrigger.whileHeld(new VerticalIndexerDownCommand(this.verticalIndexer));
    //When button 5 is pressed (Right Bumper), shoot at constant speed
    rightBumper.whileHeld(new PidShootCommand(this.shooter, 1, 1));
    //--------Turret Button Bindings--------
    //When right bumper pressed, aim at vision target if possible
    leftBumper.whileHeld(new PointTurretAtTargetWithAngleCommand(this.turret));
    //When right dpad is held, Turret Clockwise
    dPadRight.whileHeld(new SimpleTurretCWCommand(this.turret));
    //When left dpad is held, Turret Counterclockwise
    dPadLeft.whileHeld(new SimpleTurretCCWCommand(this.turret));
    //When button 9 is pressed, zero the turret
    home.whenPressed(new TurretToLimitCommand(this.turret));
    //When button 10 is pressed, get the turret out of the way for climbing
    start.whenPressed(new PrepareTurretCommand(this.turret));



    dPadUp.whileHeld(new MaxPowerShootCommand(shooter));


    //--------Shooting Button Bindings--------
    //When button 8 (Right Trigger) is pressed, start constant shooting
    //new JoystickButton(SecondaryJoystick, 5).whileHeld(new AutoShootTesting(shooter, verticalIndexer, horizontalIndexer, intake));
    //SecondaryDPadDown.whileHeld(new ConstantShootCommand(shooter));
   // SecondaryX.whileHeld(new AutoIntakeIndex(intake, horizontalIndexer, verticalIndexer));
  }

  private void addAutoChoicesToGui() {
    PossibleAutos[] enumValues = PossibleAutos.values();
    for (int i = 0; i < enumValues.length; i++) {
      autoChoice.addOption(enumValues[i].toString(), enumValues[i]);
    }
    SmartDashboard.putData(autoChoice);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    PossibleAutos choice = autoChoice.getSelected();
    switch(choice) {
      case IN_FRONT_OF_TARGET_MAX_POWER:
        return getAutoInFrontOfTarget();
      case IN_FRONT_OF_TARGET_MAX_POWER_THEN_BACK:
        return getAutoInFrontOfTargetThenBack();
      default:
        return null;
    }
  }

  private Command getAutoInFrontOfTarget() {
    //This command drives forward 4 feet when run
    Command driveForward = new DriveEncoders(1.2192, .5, driveTrain);
    //This shoots when shooter speed is over 80%
    Command shoot = new AutoShootWithHorizontal(shooter, verticalIndexer, horizontalIndexer, (int)(shooter.getMaxVelBot() * 0.80));
    //This spins up the shooter when run
    Command spinUpShooter = new ConstantShootCommand(shooter);


   //This drives and spins up, and when driving finishes, shoots for 5 seconds
   return (driveForward
            .raceWith(spinUpShooter)
          )
          .andThen(
            (shoot.withTimeout(5))
            
          );
  }
  
  private Command getAutoInFrontOfTargetThenBack() {
    //This command drives forward 4 feet when run
    Command driveForward = new DriveEncoders(1.2192, .5, driveTrain);
    //This shoots when shooter speed is over 80%
    Command shoot = new AutoShootWithHorizontal(shooter, verticalIndexer, horizontalIndexer, (int)(shooter.getMaxVelBot() * 0.80));
    //This spins up the shooter when run
    Command spinUpShooter = new ConstantShootCommand(shooter);
    //This drives back 8 feet
    Command driveBack = new DriveEncoders(-2.4384, -1, driveTrain);

   //This drives and spins up, and when driving finishes, shoots for 10 seconds
   return (driveForward
            .raceWith(spinUpShooter)
          )
          .andThen(
            (shoot.withTimeout(5))
            .andThen(
              driveBack
            )
          );
  }
}