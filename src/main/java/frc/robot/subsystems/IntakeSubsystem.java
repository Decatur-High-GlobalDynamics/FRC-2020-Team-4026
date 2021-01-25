/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Objects;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private final WPI_TalonSRX intake;

  private final double intakeSpeed = .5;
  private final double outtakeSpeed = -.5;

  public IntakeSubsystem() {
    throw new IllegalArgumentException(
        "not allowed! ctor must provide parameters for all dependencies");
  }

  public IntakeSubsystem(WPI_TalonSRX intake) {
    this.intake = Objects.requireNonNull(intake, "intake must not be null");
  }

  public static IntakeSubsystem Create() {
    WPI_TalonSRX intake = new WPI_TalonSRX(Constants.IntakeCAN);
    return new IntakeSubsystem(intake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Subsystems.Shooter.currentCMD", "" + this.getCurrentCommand());
  }

  public void inTake() {
    intake.set(intakeSpeed);
  }

  public void outTake() {
    intake.set(outtakeSpeed);
  }

  public void stop() {
    intake.set(0);
  }
}
