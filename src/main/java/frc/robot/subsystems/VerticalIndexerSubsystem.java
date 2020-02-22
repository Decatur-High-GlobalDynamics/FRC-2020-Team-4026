/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VerticalIndexerSubsystem extends SubsystemBase {
  /**
   * Creates a new VerticalIndexerSubsystem.
   */
  private final VictorSP verticalIndexer;
  private final DigitalInput lowerLimit;
  private final DigitalInput middleLimit;
  private final DigitalInput upperLimit;




  private final double upSpeed = .5;
  private final double downSpeed = -.5;
  public VerticalIndexerSubsystem() {
    verticalIndexer = new VictorSP(Constants.IndexerVertPWM);
    lowerLimit = new DigitalInput(Constants.IndexerLowerLimitDIO);
    middleLimit = new DigitalInput(Constants.IndexerMiddleLimitDIO);
    upperLimit = new DigitalInput(Constants.IndexerUpperLimitDIO);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Subsystems.VerticalIndexer.lowerLimitStatus", this.getLowerLimit());
    SmartDashboard.putBoolean("Subsystems.VerticalIndexer.middleLimitStatus", this.getMiddleLimit());
    SmartDashboard.putBoolean("Subsystems.VerticalIndexer.upperLimitStatus", this.getUpperLimit());


  }
  public boolean getLowerLimit(){
    return this.lowerLimit.get();
  }
  public boolean getMiddleLimit(){
    return this.middleLimit.get();
  }
  public boolean getUpperLimit(){
    return this.upperLimit.get();
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
