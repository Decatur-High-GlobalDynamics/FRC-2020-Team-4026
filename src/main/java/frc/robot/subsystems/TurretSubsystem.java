/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
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
  private double radPerPulse = 0.1;
 
  private boolean shouldTurnCW = true;


  private final double radAt0Ticks = Math.PI/2;

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
    shouldTurnCW = SmartDashboard.getBoolean("Subsystems.Turret.isTurnCW", true);
    SmartDashboard.putBoolean("Subsystems.Turret.isTurnCW", shouldTurnCW);
    radPerPulse = SmartDashboard.getNumber("Subsystems.Turret.radPerPulse", 0.1);
    SmartDashboard.putNumber("Subsystems.Turret.radPerPulse", radPerPulse);
    SmartDashboard.putNumber("Subsystems.Turret.turretPosition", this.getTicks());
  }

  public boolean shouldTurnCW(){
    return shouldTurnCW;
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
  

  public double convertToRad(double ticks){
    return ticks * radPerPulse + radAt0Ticks;
  }
  public double convertToTicks(double rad){
    return rad / radPerPulse;
  }

  public void rotateToPosition(double targetRad){
    double targetTicks = (shouldTurnCW ? -1: 1) * convertToTicks(targetRad);
    turretMotor.set(ControlMode.Position, targetTicks);
  }

  public void resetEncoder(){
    turretMotor.getSensorCollection().setQuadraturePosition(0, 0);
  }
  public double getTicks(){
    return turretMotor.getSensorCollection().getQuadraturePosition();
  }
  public double getRadians(){
    return turretMotor.getSensorCollection().getQuadraturePosition() * radPerPulse;
  }

  public boolean getTurretLimit(){
    return !turretLimit.get();
  }

}
