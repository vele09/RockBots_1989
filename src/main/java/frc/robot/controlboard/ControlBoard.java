package frc.robot.controlboard;

import edu.wpi.first.wpilibj2.command.button.Trigger;

///////////////////////////////////////////////////////////////////////////////
// Description: Clase para juntar las clases de los controlles
// Authors: -
// Notes: na
///////////////////////////////////////////////////////////////////////////////

public class ControlBoard {
  // Objeto para hacer el singleton
  private static ControlBoard mInstance = null;

  // Funcion para leer el singleton object
  public static ControlBoard getInstance() {
    if (mInstance == null) {
      mInstance = new ControlBoard();
    }
    return mInstance;
  }

  private IDriverControlBoard mDriverControlBoard;
  private IOperatorControlBoard mOperatorControlBoard;

  private ControlBoard() {
    mDriverControlBoard = DriverControlBoard.getInstance();
    mOperatorControlBoard = OperatorControlBoard.getInstance();
  }

  public double driverLeftStickX() {
    return mDriverControlBoard.getLeftStickX();
  }

  public double driverLeftStickY() {
    return mDriverControlBoard.getLeftStickY();
  }

  public double driverRightStickX() {
    return mDriverControlBoard.getRightStickX();
  }

  public double driverRightStickY() {
    return mDriverControlBoard.getRightStickY();
  }

  public double driverTriggersDif() {
    return mDriverControlBoard.getTriggersDif();
  }

  public Trigger driverYButton() {
    return mDriverControlBoard.getYButton();
  }

  public Trigger driverXButton() {
    return mDriverControlBoard.getXButton();
  }

  public Trigger driverBButton() {
    return mDriverControlBoard.getBButton();
  }

  public Trigger driverAButton() {
    return mDriverControlBoard.getAButton();
  }

  public Trigger driverRightBumper() {
    return mDriverControlBoard.getRightBumper();
  }

  public Trigger driverLeftBumper() {
    return mDriverControlBoard.getLeftBumper();
  }

  public Trigger driverStartButton() {
    return mDriverControlBoard.getStartButton();
  }

  public Trigger driverBackButton() {
    return mDriverControlBoard.getBackButton();
  }

  public Trigger driverBackAndStartButtons() {
      return driverBackButton().and(driverStartButton());
  }

  public Trigger driverLeftTrigger() {
    return mDriverControlBoard.getLeftTrigger();
  }

  public Trigger driverRightTrigger() {
    return mDriverControlBoard.getRightTrigger();
  }

  public double operatorLeftStickX() {
    return mOperatorControlBoard.getLeftStickX();
  }

  public double operatorLeftStickY() {
    return mOperatorControlBoard.getLeftStickY();
  }

  public double operatorRightStickX() {
    return mOperatorControlBoard.getRightStickX();
  }

  public double operatorRightStickY() {
    return mOperatorControlBoard.getRightStickY();
  }

  public double operatorTriggersDif() {
    return mOperatorControlBoard.getTriggersDif();
  }

  public Trigger operatorTriggerLeftPress() {
    return mOperatorControlBoard.getTriggerLeftPress();
  }

  public Trigger operatorTriggerRightPress() {
    return mOperatorControlBoard.getTriggerRightPress();
  }

  public Trigger operatorYButton() {
    return mOperatorControlBoard.getYButton();
  }

  public Trigger operatorXButton() {
    return mOperatorControlBoard.getXButton();
  }

  public Trigger operatorBButton() {
    return mOperatorControlBoard.getBButton();
  }

  public Trigger operatorAButton() {
    return mOperatorControlBoard.getAButton();
  }

  public Trigger operatorRightBumper() {
    return mOperatorControlBoard.getRightBumper();
  }

  public Trigger operatorLeftBumper() {
    return mOperatorControlBoard.getLeftBumper();
  }

  public Trigger operatorStartButton() {
    return mOperatorControlBoard.getStartButton();
  }

  public Trigger operatorBackButton() {
    return mOperatorControlBoard.getBackButton();
  }

  public Trigger operatorBackAndStartButtons() {
      return operatorBackButton().and(operatorStartButton());
  }

  public Trigger operatorPovLeft() {
    return mOperatorControlBoard.getPovLeft();
  }

  public Trigger operatorPovRight() {
    return mOperatorControlBoard.getPovRight();
  }

  public Trigger operatorPovDown() {
    return mOperatorControlBoard.getPovDown();
  }

  public Trigger operatorPovUp() {
    return mOperatorControlBoard.getPovUp();
  }

  public Trigger driverPovLeft() {
    return mDriverControlBoard.getPovLeft();
  }

  public Trigger driverPovRight() {
    return mDriverControlBoard.getPovRight();
  }

}