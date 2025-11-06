package com.team3478.frc2025.controlboard;

import edu.wpi.first.wpilibj2.command.button.Trigger;

///////////////////////////////////////////////////////////////////////////////
// Description: Interface con las funciones para leer los botones y sticks del control del operador.
// Authors: -
// Notes: na
///////////////////////////////////////////////////////////////////////////////

public interface IOperatorControlBoard {
  double getLeftStickX();

  double getLeftStickY();

  double getRightStickX();

  double getRightStickY();

  double getTriggersDif();

  Trigger getTriggerLeftPress();

  Trigger getTriggerRightPress();

  Trigger getYButton();

  Trigger getXButton();

  Trigger getBButton();

  Trigger getAButton();

  Trigger getRightBumper();

  Trigger getLeftBumper();

  Trigger getStartButton();

  Trigger getBackButton();

  Trigger getPovLeft();

  Trigger getPovRight();

  Trigger getPovDown();

  Trigger getPovUp();
}
