/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {
  /**
   * Creates a new TurretSubsystem.
   */
  private final WPI_TalonSRX turretMotor;
  private final DigitalInput turretLimit;

  private final double baseTurnSpeed = .15;
  private double turnSpeed = baseTurnSpeed;
  private int numEStops = 0;

  public TurretSubsystem() {
    turretMotor = new WPI_TalonSRX(Constants.TurretCAN);
    turretLimit = new DigitalInput(Constants.TurretLimitDIO);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (this.getTurretLimit() && turretMotor.get() > 0){
      this.stop();
      numEStops ++; 
    }
    turnSpeed = SmartDashboard.getNumber("Subsystems.Turret.turnSpeed", baseTurnSpeed);
    SmartDashboard.putNumber("Subsystems.Turret.turnSpeed", turnSpeed);
	  SmartDashboard.putNumber("Subsystems.Turret.motorPower", turretMotor.get());
    SmartDashboard.putBoolean("Subsystems.Turret.limitStatus", this.getTurretLimit());
    SmartDashboard.putNumber("Subsystems.Turret.numEStops", numEStops);
  }
  public void goClockwise(){
    turretMotor.set(turnSpeed);
  }
  public void goCounterClockwise(){
    turretMotor.set(-turnSpeed);
  }
  public void stop(){
    turretMotor.set(0);
  }

  public boolean getTurretLimit(){
    return !turretLimit.get();
  }

}
