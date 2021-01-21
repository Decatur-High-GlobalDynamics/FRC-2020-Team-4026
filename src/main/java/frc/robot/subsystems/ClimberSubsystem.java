/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.Objects;

public class ClimberSubsystem extends SubsystemBase {
  private final WPI_VictorSPX leftClimber;
  private final WPI_TalonSRX rightClimber;
  private final DigitalInput leftLimit;
  private final DigitalInput rightLimit;
  private final DigitalInput leftHookLimit;
  private final DigitalInput rightHookLimit;

  // constructors

  public ClimberSubsystem() {
    throw new IllegalArgumentException(
        "not allowed! ctor must provide parameters for all dependencies");
  }

  // decouple external dependencies to allow dependency injection for testing
  public ClimberSubsystem(
      WPI_VictorSPX leftClimber,
      WPI_TalonSRX rightClimber,
      DigitalInput leftLimit,
      DigitalInput rightLimit,
      DigitalInput leftHookLimit,
      DigitalInput rightHookLimit) {
    this.leftClimber = Objects.requireNonNull(leftClimber, "leftClimber must not be null");
    this.rightClimber = Objects.requireNonNull(rightClimber, "rightClimber must not be null");
    this.leftLimit = Objects.requireNonNull(leftLimit, "leftLimit must not be null");
    this.rightLimit = Objects.requireNonNull(rightLimit, "rightLimit must not be null");
    this.rightHookLimit = Objects.requireNonNull(rightHookLimit, "rightHookLimit must not be null");
    this.leftHookLimit = Objects.requireNonNull(leftHookLimit, "leftHookLimit must not be null");
  }

  // ClimberSubsystem Factory
  public static ClimberSubsystem Create() {
    WPI_VictorSPX leftClimber = new WPI_VictorSPX(Constants.LeftClimbCAN);
    WPI_TalonSRX rightClimber = new WPI_TalonSRX(Constants.RightClimbCAN);
    DigitalInput leftLimit = new DigitalInput(Constants.Climber_LeftLimitDIO);
    DigitalInput rightLimit = new DigitalInput(Constants.Climber_RightLimitDIO);
    DigitalInput leftHookLimit = new DigitalInput(Constants.Hook_LeftDIO);
    DigitalInput rightHookLimit = new DigitalInput(Constants.Hook_RightDIO);
    return new ClimberSubsystem(
        leftClimber, rightClimber, leftLimit, rightLimit, leftHookLimit, rightHookLimit);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putBoolean("Subsystem.Climber.leftLimit", leftLimit.get());
    SmartDashboard.putBoolean("Subsystems.Climber.rightLimit", rightLimit.get());

    SmartDashboard.putBoolean("Subsystems.Climber.leftHookLimit", leftHookLimit.get());
    SmartDashboard.putBoolean("Subsystems.Climber.rightHookLimit", rightHookLimit.get());
  }

  public void stop() {
    leftClimber.set(0);
    rightClimber.set(0);
  }

  public void setClimbers(double power) {
    leftClimber.set(-power);
    rightClimber.set((power) * .9);
  }

  public void setLeftClimber(double power) {
    leftClimber.set(-power);
  }

  public void setRightClimber(double power) {
    rightClimber.set(power);
  }
}
