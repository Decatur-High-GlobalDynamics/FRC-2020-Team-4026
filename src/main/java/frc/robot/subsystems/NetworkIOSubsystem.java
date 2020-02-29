/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NetworkIOSubsystem extends SubsystemBase {
  NetworkTableInstance inst;

  /**
   * Creates a new NetworkIOSubsystem.
   */
  public NetworkIOSubsystem() {
    inst = NetworkTableInstance.getDefault();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  //This is a function to send data down network table.
  //tableName: Name of the table to send to
  //key: the key to send with
  //value: the thing to send
  public void send(String tableName, String key, Object value) {
    inst.getTable(tableName).getEntry(key).setValue(value);
  }

  public Object get(String tableName, String key) {
    return inst.getTable(tableName).getEntry(key).getValue().getValue();
  }
}
