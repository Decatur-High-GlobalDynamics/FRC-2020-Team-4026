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

public class TurretSubsystem extends SubsystemBase {
  /**
   * Creates a new IntakeSubsystem.
   */
  private final WPI_TalonSRX turret;

  private final double turnSpeed = .25;
  public TurretSubsystem() {
    turret = new WPI_TalonSRX(Constants.TurretCan);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void goLeft(){

      turret.set(turnSpeed);
  }
  public void goRight(){
      turret.set(-turnSpeed);
  }
  public void stop(){
  turret.set(0);
  }

}
