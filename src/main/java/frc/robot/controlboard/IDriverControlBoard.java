package frc.robot.controlboard;

import edu.wpi.first.wpilibj2.command.button.Trigger;

///////////////////////////////////////////////////////////////////////////////
// Description: Interface con las funciones para leer los botones y sticks del control del driver.
// Authors: -
// Notes: na
///////////////////////////////////////////////////////////////////////////////

public interface IDriverControlBoard {
  double getLeftStickX();

  double getLeftStickY();

  double getRightStickX();

  double getRightStickY();

  double getTriggersDif();

  Trigger getYButton();

  Trigger getXButton();

  Trigger getBButton();

  Trigger getAButton();

  Trigger getRightBumper();

  Trigger getLeftBumper();

  Trigger getStartButton();

  Trigger getBackButton();

  Trigger getLeftTrigger();

  Trigger getRightTrigger();
  
  Trigger getPovLeft();

  Trigger getPovRight();
}
