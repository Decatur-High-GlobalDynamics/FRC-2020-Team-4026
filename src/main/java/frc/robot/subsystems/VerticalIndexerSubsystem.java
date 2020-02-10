/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VerticalIndexerSubsystem extends SubsystemBase {
  /**
   * Creates a new VerticalIndexerSubsystem.
   */
  private final VictorSP verticalIndexer;

  private final double upSpeed = .5;
  private final double downSpeed = -.5;
  public VerticalIndexerSubsystem() {
    verticalIndexer = new VictorSP(Constants.IndexerVertPWM);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
