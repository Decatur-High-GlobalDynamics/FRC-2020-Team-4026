/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;
import frc.robot.PidParameters;
import frc.robot.TeamTalonSRX;
import frc.robot.TeamUtils;
import frc.robot.commands.PrepareTurretCommand;
import frc.robot.commands.TurretToLimitCommand;

public class TurretSubsystem extends SubsystemBase {
  /**
   * Creates a new TurretSubsystem.
   */
  private final TeamTalonSRX turretMotor;
  private final DigitalInput turretLimit;

  private final double baseTurnSpeed = .1;
  private double maxTurnSpeed = baseTurnSpeed;

  private static final boolean sensorPhase = true;
  private static final boolean motorInvert = true;
  
  private boolean hasBeenCalibrated=false;

  private final double MinPowerToMove = 0.0425;

  private final int stallThresh = 30;

  private double lastGoodAngle;
  private int numSequentialErrors = 0;

  public boolean isTurretToLimitRunning = false;
  
  private final PidParameters pidParams = new PidParameters(0.35, 0.05, 0.1, 0, 0, 0.15, 10);

  //Number of encoder ticks to go when rotating
  private int rotationSpeed = 500;

  // Location (based on limitswitch = 0) of far clockwise range
  private long minEncoderRange = -7000;

  // How many radians per encoder tick
  // Found as follows: Straight ahead was -2914, Facing right was -6798
  // These PI/2 radians took (6798-2914) ticks
  private double radPerPulse = Math.PI/2 / (6798-2914);

  // What is encoder value for Straight Ahead (aka Pi/2 radians)
  private final double ticksAtPiOver2Rads = -2914;

  public TurretSubsystem() {
    turretMotor = new TeamTalonSRX("Subsystems.Turret.motor", Constants.TurretCAN);
    turretMotor.configFactoryDefault();

    turretLimit = new DigitalInput(Constants.TurretLimitDIO);

    turretMotor.setNeutralMode(NeutralMode.Brake);
    turretMotor.setInverted(motorInvert);
    turretMotor.setSensorPhase(sensorPhase);
    turretMotor.configNominalOutputForward(MinPowerToMove);
    turretMotor.configNominalOutputReverse(-MinPowerToMove);
    turretMotor.configureWithPidParameters(pidParams, 0);

    turretMotor.configReverseSoftLimitEnable(false);
    turretMotor.configForwardSoftLimitEnable(false);

    turretMotor.setNeutralMode(NeutralMode.Brake);


  }
  


  private boolean isPowerOkay(double powerToCheck) {
    // Always safe to stop!
    if (powerToCheck==0)
      return true;

    // Check safety limits if turret is not running TurretToLimit calibration
    Command cmd = getCurrentCommand();
    if (isTurretToLimitRunning) {
      return true;
    }

    if ( !hasBeenCalibrated ) {
      System.err.println("Turret has not been calibrated! Press home on controller 2!");
      return false;
    }

    if (this.getTicks() > 0 && powerToCheck > 0) {
      return false;
    }

    if (this.getTicks() < minEncoderRange && powerToCheck < 0){
      return false;
    }

    return true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Command cmd = getCurrentCommand();
    
    if (!isPowerOkay(turretMotor.getMotorOutputPercent())) {
        turretMotor.noteEmergencyStop();
        stop();
        if (cmd != null) 
          cmd.cancel();

        stop();
      }

    if ( cmd == null )
      SmartDashboard.putString("Subsystems.Turret.Command", "none");
    else
      SmartDashboard.putString("Subsystems.Turret.Command", cmd.toString());
 
    maxTurnSpeed = SmartDashboard.getNumber("Subsystems.Turret.maxTurnSpeed", maxTurnSpeed);
    SmartDashboard.putNumber("Subsystems.Turret.maxTurnSpeed", maxTurnSpeed);

    SmartDashboard.putBoolean("Subsystems.Turret.limitSwitch", this.getTurretLimitSwitch());
    SmartDashboard.putBoolean("Subsystems.Turret.hasBeenCalibrate", hasBeenCalibrated);

    minEncoderRange = (long) SmartDashboard.getNumber("Subsystems.Turret.minEncoderRange", minEncoderRange);
    SmartDashboard.putNumber("Subsystems.Turret.minEncoderRange", minEncoderRange);

    radPerPulse = SmartDashboard.getNumber("Subsystems.Turret.radPerPulse", radPerPulse);
    SmartDashboard.putNumber("Subsystems.Turret.radPerPulse", radPerPulse);
    SmartDashboard.putNumber("Subsystems.Turret.turretPosition-rads", this.getRadians());

    turretMotor.periodic();
    // Display and Update PID parameters
    pidParams.periodic("Subsystems.Turret", turretMotor, 0);
    PidParameters previousPidParameters = pidParams.clone();

    pidParams.kF = SmartDashboard.getNumber("Subsystems.Turret.kF", pidParams.kF);
    SmartDashboard.putNumber("Subsystems.Turret.kF", pidParams.kF);
    pidParams.kP = SmartDashboard.getNumber("Subsystems.Turret.kP", pidParams.kP);
    SmartDashboard.putNumber("Subsystems.Turret.kP", pidParams.kP);
    pidParams.kI = SmartDashboard.getNumber("Subsystems.Turret.kI", pidParams.kI);
    SmartDashboard.putNumber("Subsystems.Turret.kI", pidParams.kI);
    pidParams.kD = SmartDashboard.getNumber("Subsystems.Turret.kD", pidParams.kD);
    SmartDashboard.putNumber("Subsystems.Turret.kD", pidParams.kD);
    pidParams.kPeakOutput = SmartDashboard.getNumber("Subsystems.Turret.kPeakOutput", pidParams.kPeakOutput);
    SmartDashboard.putNumber("Subsystems.Turret.kPeakOutput", pidParams.kPeakOutput);
    pidParams.errorTolerance = (int) SmartDashboard.getNumber("Subsystems.Turret.errorTolerance", pidParams.errorTolerance);
    SmartDashboard.putNumber("Subsystems.Turret.errorTolerance", pidParams.errorTolerance);
    // If the pidParameters have changed, load them into motor
    if ( ! previousPidParameters.equals(pidParams) ) {
      turretMotor.configureWithPidParameters(pidParams,0);
    }
    SmartDashboard.putNumber("Subsystems.Turret.xAngleAdjusted", this.getVisionXAngle());

    SmartDashboard.putNumber("Subsystems.Turret.sensorPosition", turretMotor.getSelectedSensorPosition(0));
    SmartDashboard.putString("Subsystems.Turret.Mode", turretMotor.getControlMode().toString());
    if ( turretMotor.getControlMode() == ControlMode.Position) {
      SmartDashboard.putNumber("Subsystems.Turret.targetPosition", turretMotor.getClosedLoopTarget(0));
      SmartDashboard.putNumber("Subsystems.Turret.error", turretMotor.getClosedLoopError(0));
    } else {
      SmartDashboard.putNumber("Subsystems.Turret.targetPosition", 0);
      SmartDashboard.putNumber("Subsystems.Turret.error", 0);
    }
    if(!hasBeenCalibrated && !(isTurretToLimitRunning) ){
     // new TurretToLimitCommand(this).schedule();
    }
  }
 
  public void goClockwise(double power){
    // We'll negate it as necessary
    power = Math.abs(power);
    power = Math.min(power, maxTurnSpeed);

    turretMotor.configPeakOutputReverse(-power);
    if ( !isPowerOkay(-power) ) {
      turretMotor.noteEmergencyStop();
      return;
    }

    turretMotor.set(ControlMode.PercentOutput, -power);
  }

  public void positionRotateCW(){
    this.startRotatingToEncoderPosition(this.getTicks() - rotationSpeed);
  }
  public void positonRotateCCW(){
    this.startRotatingToEncoderPosition(this.getTicks() + rotationSpeed);
  }

  public void goClockwise(){
    goClockwise(maxTurnSpeed);
  }

  public void goCounterClockwise(double power){
    // We'll take care of the sign as necessary
    power = Math.abs(power);
    power = Math.min(power, maxTurnSpeed);

    turretMotor.configPeakOutputForward(power);
    if ( !isPowerOkay(power) ) {
      turretMotor.noteEmergencyStop();
      return;
    }

    turretMotor.set(ControlMode.PercentOutput, power);
  }

  public void goCounterClockwise(){
    goCounterClockwise(maxTurnSpeed);
  }

  public void markAsCalibrated() {
    hasBeenCalibrated=true;
  }
  public boolean checkCalibration(){
    return hasBeenCalibrated;
  }

  public double getSpeed(){
    return turretMotor.getSelectedSensorVelocity();
  }

  public boolean isStalled(){
    return this.getSpeed() <= stallThresh; 
  }

  public void stop(){
    // Set a little power in the opposite direction that we were heading
     turretMotor.set(ControlMode.PercentOutput, -0.004026*Math.signum(turretMotor.getMotorOutputPercent()));
     // Tell motor to hold the position
     startRotatingToEncoderPosition((long)MathUtil.clamp(turretMotor.getSelectedSensorPosition(), minEncoderRange,0));
  }

  public boolean isMotorBusy() {
    if ( turretMotor.getControlMode() == ControlMode.Position )
      return Math.abs(turretMotor.getClosedLoopError()) < pidParams.errorTolerance;
    else if ( turretMotor.getControlMode() == ControlMode.PercentOutput)
      return getPower() == 0;
    else 
      return false;
  }

  public double convertToRad(long ticks){
    // Find how far ticks is from Pi/2 and scale it
    return Math.PI/2 + (ticks - ticksAtPiOver2Rads) * radPerPulse;
  }
  public double convertToTicks(double rad){
    return ticksAtPiOver2Rads + (rad - Math.PI/2)/radPerPulse;
  }

  public void startRotatingToEncoderPosition(long encoderPosition) {
    turretMotor.configureWithPidParameters(pidParams, 0);
    long reqPosition = encoderPosition;
    if (!(this.getCurrentCommand() instanceof PrepareTurretCommand)){
      reqPosition = (long) MathUtil.clamp(reqPosition, minEncoderRange, 0);
    }
    turretMotor.set(ControlMode.Position, reqPosition);
  }

  public void startRotatingToPosition(double targetRad){
    long targetTicks =  Math.round(convertToTicks(targetRad));
    startRotatingToEncoderPosition(targetTicks);
  }

  public void resetEncoder(){ turretMotor.resetEncoder(); }
  public long getTicks(){
    return turretMotor.getCurrentEncoderValue();
  }
  public double getRadians(){
    return convertToRad(getTicks());
  }
  public double getPower(){
    return turretMotor.getMotorOutputPercent();
  }

  public boolean getTurretLimitSwitch(){
    return !turretLimit.get();
  }

  public double getVisionXAngle(){
    Object result = TeamUtils.getFromNetworkTable("angles", "xAngle");
    if (result != null && (Double) result != 4026.0){
      lastGoodAngle = (Double) result;
      numSequentialErrors = 0;
    } else {
      numSequentialErrors ++;
    }
    if (numSequentialErrors > 10){
      return 4026;
    } else {
      return lastGoodAngle;
    }
  }

}
