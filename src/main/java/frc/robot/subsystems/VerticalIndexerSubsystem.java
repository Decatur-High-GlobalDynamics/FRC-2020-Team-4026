/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.TeamTalonSRX;

public class VerticalIndexerSubsystem extends SubsystemBase {
  /**
   * Creates a new VerticalIndexerSubsystem.
   */
  private final TeamTalonSRX verticalIndexer;

  private DigitalInput bottomSwitchA = new DigitalInput(Constants.VerticalIndexer_BottomLimit_DIO_A);
  private DigitalInput bottomSwitchB = new DigitalInput(Constants.VerticalIndexer_BottomLimit_DIO_B);
  private DigitalInput bottomSwitchC = new DigitalInput(Constants.VerticalIndexer_BottomLimit_DIO_C);
  private DigitalInput middleSwitch = new DigitalInput(Constants.VerticalIndexer_MiddleLimit_DIO);
  private DigitalInput topSwitch = new DigitalInput(Constants.VerticalIndexer_TopLimit_DIO);

  public int ticksUntilTransfered = 6000;

  private final double upSpeed = .25;
  private final double downSpeed = -.5;
  public VerticalIndexerSubsystem() {
    verticalIndexer = new TeamTalonSRX("Subsystems.VerticalIndexer.VIndxMotor", Constants.IndexerVertCAN);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    verticalIndexer.periodic();
    SmartDashboard.putBoolean("Subsystems.VerticalIndexer.TopSwitch", topSwitchIsPressed());
    SmartDashboard.putBoolean("Subsystems.VerticalIndexer.MiddleSwitch", middleSwitchIsPressed());
    SmartDashboard.putBoolean("Subsystems.VerticalIndexer.BottomSwitch", bottomSwitchIsPressed());
    ticksUntilTransfered = (int) SmartDashboard.getNumber("Subsystems.VerticalIndexer.TicksUntilTransferred", ticksUntilTransfered);
    SmartDashboard.putNumber("Subsystems.VerticalIndexer.TicksUntilTransferred", ticksUntilTransfered);
    SmartDashboard.putNumber("Subsystems.VerticalIndexer.currentPosition", verticalIndexer.getSelectedSensorPosition());
  }

  public boolean topSwitchIsPressed() {
    // Negative because of opposite switch polarity
    return !topSwitch.get();
  }
  public boolean middleSwitchIsPressed() {
    // Negative because of opposite switch polarity
    return !middleSwitch.get();
  }
  public boolean bottomSwitchIsPressed() {
    // Negative because of opposite switch polarity

    // The bottomSwitchB is wired backwards because the other half
    // of the limit switch is broken!!
    return !bottomSwitchA.get() || bottomSwitchB.get() || !bottomSwitchC.get();
  }

  public int getPosition(){
    return Math.abs(verticalIndexer.getSelectedSensorPosition());
  }

  public void up(){
      verticalIndexer.set(upSpeed);
  }
  public void down(){
      verticalIndexer.set(downSpeed);
  }
  public void stop(){
  verticalIndexer.set(0);
  }

}
