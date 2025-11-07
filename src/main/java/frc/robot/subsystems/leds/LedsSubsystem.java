package frc.robot.subsystems.leds;

///////////////////////////////////////////////////////////////////////////////
// Description: Esta clase es la encargada de los Leds del robot.
// Notes:
//  - Una sola tira de 44 LEDs controlada desde una salida PWM del roborio
//  - Distribucion: [Left Edge: 10 LEDs][Center: 24 LEDs][Right Edge: 10 LEDs]
//  - Los leds los controlamos desde una salida de pwm del roborio.
///////////////////////////////////////////////////////////////////////////////

import frc.robot.Constants;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedsSubsystem extends SubsystemBase {

  // Colores de los Leds
  public enum Colors {
    Transparent,
    Red,
    Yellow,
    Green,
    Aqua,
    Blue,
    Magenta,
    White,
    Purple,
    Black,
    Orange
  }

  // Secciones de los Leds
  public enum Section {
    Left,
    Center,
    Right,
    Edges,
    All
  }

  // Tira de leds unica
  private AddressableLED mLeds;
  /// Buffer para controlar los leds
  private AddressableLEDBuffer mLedBuffer;
  /// Variables para almacenar el ultimo estado de cada seccion
  private Colors cacheColorCenter = Colors.Transparent;
  private Colors cacheColorEdgeLeft = Colors.Transparent;
  private Colors cacheColorEdgeRight = Colors.Transparent;
  // Bandera para saber si los leds necesitan cambio.
  private boolean ledsChanged = false;
  // Bandera para saber si los leds estan corriendo en modo simulacion
  private boolean isSimulation = false;

  // Constructor del subsistema
  public LedsSubsystem(boolean isSimulation) {
    this.isSimulation = isSimulation;
    if(!isSimulation){
      // Init Leds object
      mLeds = new AddressableLED(Constants.Leds.kPwmPort);
      // Init Leds buffer para una sola tira
      mLedBuffer = new AddressableLEDBuffer(Constants.Leds.kLeds);
      mLeds.setLength(mLedBuffer.getLength());
      // Aseguramos esten apagados los leds
      turnOff();
    }
  } 

  @Override
    public void periodic() {
        if (ledsChanged && !isSimulation) {
            mLeds.setData(mLedBuffer);
            mLeds.start();
            ledsChanged = false;
        }
    }

  // Funcion para apagar una tira o todas las tiras de leds
  public void turnOff() {
    if(isSimulation) return;
    setColor(Section.All, Colors.Black);
    mLeds.stop();
  }
  
  // Funcion para cambiar la seccion del centro
  private void setCenter(int[] colorvals) {
    for (var i = Constants.Leds.kLedsEdge; i < (Constants.Leds.kLeds - Constants.Leds.kLedsEdge); i++) {
      mLedBuffer.setRGB(i, colorvals[0], colorvals[1], colorvals[2]);
    }
  }

  // Funcion para cambiar la orilla de la izquierda
  private void setLeftEdge(int[] colorvals) {
    for (var i = 0; i < Constants.Leds.kLedsEdge; i++) {
      mLedBuffer.setRGB(i, colorvals[0], colorvals[1], colorvals[2]);
    }
  }

  // Funcion para cambiar la orilla de la derecha
  private void setRightEdge(int[] colorvals) {
    for (var i = (Constants.Leds.kLeds - Constants.Leds.kLedsEdge); i < Constants.Leds.kLeds; i++) {
      mLedBuffer.setRGB(i, colorvals[0], colorvals[1], colorvals[2]);
    }
  }

  // Funcion para obtener el color en rgb
  private int[] getColor(Colors colorx) {
    int[] colorvals = new int[] {0, 0, 0};
    switch (colorx) {
      case Red:
        colorvals = new int[] {255, 0, 0};
        break;
      case Orange:
        colorvals = new int[] {255, 156, 18};
        break;
      case Yellow:
        colorvals = new int[] {255, 255, 0};
        break;
      case Green:
        colorvals = new int[] {0, 255, 0};
        break;
      case Aqua:
        colorvals = new int[] {0, 255, 255};
        break;
      case Blue:
        colorvals = new int[] {0, 0, 255};
        break;
      case Magenta:
        colorvals = new int[] {255, 0, 255};
        break;
      case White:
        colorvals = new int[] {255, 255, 255};
        break;
      case Purple:
        colorvals = new int[] {140, 0, 255};
        break;
      default:
        // nada
        break;
    }
    return (colorvals);
  }

  /// Funcion para actualizar el color de la tira de leds
  public void setColor(Section sectionx, Colors colorx) {
    if(isSimulation) return;

    // Banderas para marcar que seccion necesita cambio
    boolean needChangeLeft = false;
    boolean needChangeCenter = false;
    boolean needChangeRight = false;

    // Revisamos las secciones que involucren cambio en la edge izquierda
    if (sectionx == Section.All || sectionx == Section.Edges || sectionx == Section.Left) {
      if (cacheColorEdgeLeft != colorx) {
        cacheColorEdgeLeft = colorx;
        needChangeLeft = true;
      }
    }

    // Revisamos las secciones que involucren cambio al centro
    if (sectionx == Section.All || sectionx == Section.Center) {
      if (cacheColorCenter != colorx) {
        cacheColorCenter = colorx;
        needChangeCenter = true;
      }
    }

    // Revisamos las secciones que involucren cambio en la edge derecha
    if (sectionx == Section.All || sectionx == Section.Edges || sectionx == Section.Right) {
      if (cacheColorEdgeRight != colorx) {
        cacheColorEdgeRight = colorx;
        needChangeRight = true;
      }
    }

    // Aplicamos los cambios solo si es necesario
    if (needChangeLeft || needChangeCenter || needChangeRight) {
      int[] colorvals = getColor(colorx);

      if (needChangeLeft) {
        setLeftEdge(colorvals);
      }

      if (needChangeCenter) {
        setCenter(colorvals);
      }

      if (needChangeRight) {
        setRightEdge(colorvals);
      }

      ledsChanged = true;
    }
  }

  // Comando para actualizar los leds
  public Command UpdateLeds(Colors color, Section section){
    return runOnce(()->{
        setColor(section,color);
    });
  }

  // Comando para actualizar los leds
  public Command StopLeds(){
    return runOnce(()->{
        turnOff();
    });
  }

} 