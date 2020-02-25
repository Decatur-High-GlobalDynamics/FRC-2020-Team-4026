/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Gains;
import frc.robot.commands.StopTurret;
import frc.robot.commands.TurretToLimit;

public class TurretSubsystem extends SubsystemBase {
  /**
   * Creates a new TurretSubsystem.
   */
  private final TalonSRX turretMotor;
  private final DigitalInput turretLimit;

  private final double baseTurnSpeed = .1;
  private double turnSpeed = baseTurnSpeed;
  private int numEStops = 0;
  private double radPerPulse = 0.1;

  private static final boolean sensorPhase = true;
  private static final boolean motorInvert = true;
  

  private final double MinPowerToMove = 0.0425;
  private final int ErrorTolerance = 25;

  
  private static final Gains gains = new Gains(0.025, 0, 1, 0, 0, 0.1);
 
  private boolean isResetting = false;


  private final double radAt0Ticks = Math.PI/2;

  public TurretSubsystem() {
    turretMotor = new WPI_TalonSRX(Constants.TurretCAN);
    turretLimit = new DigitalInput(Constants.TurretLimitDIO);
    turretMotor.setNeutralMode(NeutralMode.Brake);
    turretMotor.setInverted(motorInvert);
    turretMotor.configFactoryDefault();
    turretMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 20);
    turretMotor.setSensorPhase(sensorPhase);
    turretMotor.configNominalOutputForward(MinPowerToMove);
    turretMotor.configNominalOutputReverse(-MinPowerToMove);
    turretMotor.configPeakOutputForward(baseTurnSpeed);
    turretMotor.configPeakOutputReverse(-baseTurnSpeed);
    turretMotor.configAllowableClosedloopError(0, ErrorTolerance, 20);
    turretMotor.config_kF(0, gains.kF);
    turretMotor.config_kP(0, gains.kP);
    turretMotor.config_kI(0, gains.kI);
    turretMotor.config_kD(0, gains.kD);

    turretMotor.configReverseSoftLimitEnable(false);
    turretMotor.configForwardSoftLimitEnable(false);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if ((this.getTicks() > 0 && turretMotor.getMotorOutputPercent() > 0) && this.getCurrentCommand() instanceof TurretToLimit){
      new StopTurret(this);
      numEStops++;
    }
    if ((this.getTicks() < -6000 && turretMotor.getMotorOutputPercent() < 0) && this.getCurrentCommand() instanceof TurretToLimit){
      new StopTurret(this);
      numEStops++;
    }
    if (Math.abs(turretMotor.getMotorOutputPercent()) == 0.01){
      turretMotor.set(ControlMode.PercentOutput, 0);
    }
 
    turnSpeed = SmartDashboard.getNumber("Subsystems.Turret.turnSpeed", baseTurnSpeed);
    SmartDashboard.putNumber("Subsystems.Turret.turnSpeed", turnSpeed);
	  SmartDashboard.putNumber("Subsystems.Turret.motorPower", turretMotor.getMotorOutputPercent());
    SmartDashboard.putBoolean("Subsystems.Turret.limitStatus", this.getTurretLimit());
    SmartDashboard.putNumber("Subsystems.Turret.numEStops", numEStops);
    radPerPulse = SmartDashboard.getNumber("Subsystems.Turret.radPerPulse", 0.1);
    SmartDashboard.putNumber("Subsystems.Turret.radPerPulse", radPerPulse);
    SmartDashboard.putNumber("Subsystems.Turret.turretPosition", this.getTicks());
    SmartDashboard.putBoolean("Subsystems.Turret.isResetting", isResetting);

    SmartDashboard.putString("Subsystems.Turret.Mode", turretMotor.getControlMode().toString());
    gains.kF = SmartDashboard.getNumber("Subsystems.Turret.kF", gains.kF);
    SmartDashboard.putNumber("Subsystems.Turret.kF", gains.kF);
    gains.kP = SmartDashboard.getNumber("Subsystems.Turret.kP", gains.kP);
    SmartDashboard.putNumber("Subsystems.Turret.kP", gains.kP);
    gains.kI = SmartDashboard.getNumber("Subsystems.Turret.kI", gains.kI);
    SmartDashboard.putNumber("Subsystems.Turret.kI", gains.kI);
    gains.kD = SmartDashboard.getNumber("Subsystems.Turret.kD", gains.kD);
    SmartDashboard.putNumber("Subsystems.Turret.kD", gains.kD);

    SmartDashboard.putNumber("Subsystems.Turret.targetPosition", turretMotor.getClosedLoopTarget(0));
    SmartDashboard.putNumber("Subsystems.Turret.sensorPosition", turretMotor.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("Subsystems.Turret.error", turretMotor.getClosedLoopError(0));
  }

  public void goClockwise(){
    turretMotor.set(ControlMode.PercentOutput, -turnSpeed);
  }
  public void goCounterClockwise(){
    turretMotor.set(ControlMode.PercentOutput, turnSpeed);
  }
  public void stop(){
     turretMotor.set(ControlMode.PercentOutput, -(Math.signum(turretMotor.getMotorOutputPercent())) *0.01);
  }
  public void toPosition(int targetTicks){
    turretMotor.set(ControlMode.Position, targetTicks);
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
    return turretMotor.getMotorOutputPercent();
  }

  public boolean getTurretLimit(){
    return !turretLimit.get();
  }

}
