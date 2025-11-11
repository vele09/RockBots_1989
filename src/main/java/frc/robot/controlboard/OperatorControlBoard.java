package frc.robot.controlboard;

///////////////////////////////////////////////////////////////////////////////
// Description: Clase con las funciones para leer los botones y sticks del control del operador.
// Authors: -
// Notes: na
///////////////////////////////////////////////////////////////////////////////

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OperatorControlBoard implements IOperatorControlBoard {

  private static OperatorControlBoard mInstance = null;

  public static OperatorControlBoard getInstance() {
    if (mInstance == null) {
      mInstance = new OperatorControlBoard();
    }
    return mInstance;
  }

  private final CommandXboxController mController;

  private OperatorControlBoard() {
    mController = new CommandXboxController(Constants.OperatorConstants.kOperatorControlPort);
  }

  @Override
  public double getLeftStickX() {
    return mController.getLeftX();
  }

  @Override
  public double getLeftStickY() {
    return mController.getLeftY();
  }

  @Override
  public double getRightStickX() {
    return mController.getRightX();
  }

  @Override
  public double getRightStickY() {
    return mController.getRightY();
  }

  @Override
  public double getTriggersDif() {
    return mController.getRightTriggerAxis()-mController.getLeftTriggerAxis();
  }

  @Override
  public Trigger getTriggerLeftPress() {
    return new Trigger(()-> mController.getLeftTriggerAxis()>0.5);
  }

  @Override
  public Trigger getTriggerRightPress() {
    return new Trigger(()-> mController.getRightTriggerAxis()>0.5);
  }

  @Override
  public Trigger getYButton() {
    return mController.y();
  }

  @Override
  public Trigger getXButton() {
    return mController.x();
  }

  @Override
  public Trigger getBButton() {
    return mController.b();
  }

  @Override
  public Trigger getAButton() {
    return mController.a();
  }

  @Override
  public Trigger getRightBumper() {
    return mController.rightBumper();
  }

  @Override
  public Trigger getLeftBumper() {
    return mController.leftBumper();
  }

  @Override
  public Trigger getStartButton() {
    return mController.start();
  }

  @Override
  public Trigger getBackButton() {
    return mController.back();
  }

  @Override
  public Trigger getPovLeft() {
    return mController.povLeft();
  }

  @Override
  public Trigger getPovRight() {
    return mController.povRight();
  }

  @Override
  public Trigger getPovDown() {
    return mController.povDown();
  }

  @Override
  public Trigger getPovUp() {
    return mController.povUp();
  }
}
