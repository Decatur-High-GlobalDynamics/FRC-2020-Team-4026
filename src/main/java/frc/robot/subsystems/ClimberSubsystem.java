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
import frc.robot.constants.Ports;

public class ClimberSubsystem extends SubsystemBase {

  private final WPI_VictorSPX leftClimber;
  private final WPI_TalonSRX rightClimber;

  private final DigitalInput leftLimit;
  private final DigitalInput rightLimit;

  private final DigitalInput leftHookLimit;
  private final DigitalInput rightHookLimit;
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    leftClimber = new WPI_VictorSPX(Ports.LeftClimbCAN);
    rightClimber = new WPI_TalonSRX(Ports.RightClimbCAN);

    leftLimit = new DigitalInput(Ports.Climber_LeftLimitDIO);
    rightLimit = new DigitalInput(Ports.Climber_RightLimitDIO);

    rightHookLimit = new DigitalInput(Ports.Hook_RightDIO);
    leftHookLimit = new DigitalInput(Ports.Hook_LeftDIO);
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
