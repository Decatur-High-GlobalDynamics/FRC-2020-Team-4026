/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

  private final WPI_VictorSPX leftClimber;
  private final WPI_TalonSRX rightClimber;
  /**
   * Creates a new ClimberSubsystem.
   */
  public ClimberSubsystem() {
    leftClimber = new WPI_VictorSPX(Constants.LeftClimbCAN);
    rightClimber = new WPI_TalonSRX(Constants.RightClimbCAN);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void stop(){
    leftClimber.set(0);
    rightClimber.set(0);
  }

  public void setClimbers(double power){
    leftClimber.set(power);
    rightClimber.set(power);
  }
}
