/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private final WPI_TalonSRX shooter_bottom;
  private final WPI_TalonSRX shooter_top;
  /**
   * Creates a new ShooterSubsystem.
   */
  public ShooterSubsystem() {
    shooter_bottom = new WPI_TalonSRX(Constants.BotShooterMotorCAN);
    shooter_top = new WPI_TalonSRX(Constants.TopShooterMotorCAN);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setBottomMotor(double speed){
    this.shooter_bottom.set(speed);
  }
  public void setTopMotor(double speed){
    this.shooter_top.set(speed);
  }
  public void stop(){
    shooter_bottom.set(0);
    shooter_top.set(0);
  }
}
