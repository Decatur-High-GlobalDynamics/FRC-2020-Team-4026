package org.decaturrobotics.helpers;

import edu.wpi.first.wpilibj2.command.button.Button;

public class MockButton extends Button {

  private boolean pushed = false;

  @Override
  public boolean get() {
    return pushed; // Auto-generated method stub
  }

  public void push() {
    pushed = true;
  }

  public void release() {
    pushed = false;
  }
}
