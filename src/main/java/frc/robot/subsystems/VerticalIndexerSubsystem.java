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

public class VerticalIndexerSubsystem extends SubsystemBase {
  /**
   * Creates a new VerticalIndexerSubsystem.
   */
  private final WPI_TalonSRX verticalIndexer;

  private DigitalInput bottomSwitch = new DigitalInput(Constants.VerticalIndexer_BottomLimit_DIO);
  private DigitalInput middleSwitch = new DigitalInput(Constants.VerticalIndexer_MiddleLimit_DIO);
  private DigitalInput topSwitch = new DigitalInput(Constants.VerticalIndexer_TopLimit_DIO);

  private final double upSpeed = .5;
  private final double downSpeed = -.5;
  public VerticalIndexerSubsystem() {
    verticalIndexer = new WPI_TalonSRX(Constants.IndexerVertCAN);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putBoolean("Subsystems.VerticalIndexer.TopSwitch", topSwitchIsPressed());
    SmartDashboard.putBoolean("Subsystems.VerticalIndexer.MiddleSwitch", middleSwitchIsPressed());
    SmartDashboard.putBoolean("Subsystems.VerticalIndexer.BottomSwitch", bottomSwitchIsPressed());
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
    return !bottomSwitch.get();
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
