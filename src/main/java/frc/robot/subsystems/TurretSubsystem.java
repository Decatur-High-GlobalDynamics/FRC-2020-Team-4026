/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.StopTurret;
import frc.robot.commands.TurretToLimit;

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
 
  private boolean isResetting = false;


  private final double radAt0Ticks = Math.PI/2;

  public TurretSubsystem() {
    turretMotor = new WPI_TalonSRX(Constants.TurretCAN);
    turretLimit = new DigitalInput(Constants.TurretLimitDIO);
    turretMotor.setNeutralMode(NeutralMode.Brake);
    turretMotor.setInverted(true);

    turretMotor.configReverseSoftLimitEnable(false);
    turretMotor.configForwardSoftLimitEnable(false);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if ((this.getTicks() > 0 && turretMotor.get() > 0) && this.getCurrentCommand() instanceof TurretToLimit){
      new StopTurret(this);
      numEStops++;
    }
    if ((this.getTicks() < -6000 && turretMotor.get() < 0) && this.getCurrentCommand() instanceof TurretToLimit){
      new StopTurret(this);
      numEStops++;
    }
    if (Math.abs(turretMotor.get()) == 0.01){
      turretMotor.set(0);
    }
 
    turnSpeed = SmartDashboard.getNumber("Subsystems.Turret.turnSpeed", baseTurnSpeed);
    SmartDashboard.putNumber("Subsystems.Turret.turnSpeed", turnSpeed);
	  SmartDashboard.putNumber("Subsystems.Turret.motorPower", turretMotor.get());
    SmartDashboard.putBoolean("Subsystems.Turret.limitStatus", this.getTurretLimit());
    SmartDashboard.putNumber("Subsystems.Turret.numEStops", numEStops);
    radPerPulse = SmartDashboard.getNumber("Subsystems.Turret.radPerPulse", 0.1);
    SmartDashboard.putNumber("Subsystems.Turret.radPerPulse", radPerPulse);
    SmartDashboard.putNumber("Subsystems.Turret.turretPosition", this.getTicks());
    SmartDashboard.putBoolean("Subsystems.Turret.isResetting", isResetting);
  }

  public void goClockwise(){
    turretMotor.set(-turnSpeed);
  }
  public void goCounterClockwise(){
    turretMotor.set(turnSpeed);
  }
  public void stop(){
     turretMotor.set(-(Math.signum(turretMotor.get())) *0.01);
  }
  

  public void toggleReset(){
    isResetting = !isResetting;
  }

  public double convertToRad(double ticks){
    return ticks * radPerPulse + radAt0Ticks;
  }
  public double convertToTicks(double rad){
    return rad / radPerPulse;
  }

  public void rotateToPosition(double targetRad){
    double targetTicks =  -1 * convertToTicks(targetRad);
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
  public double getPower(){
    return turretMotor.get();
  }

  public boolean getTurretLimit(){
    return !turretLimit.get();
  }

}
