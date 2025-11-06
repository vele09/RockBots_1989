package com.team3478.lib.util;

import edu.wpi.first.math.geometry.Pose2d;

///////////////////////////////////////////////////////////////////////////////
// Description: Clase con funciones utiles generales.
// Authors: -
// Notes: na
///////////////////////////////////////////////////////////////////////////////

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.List;
import java.util.StringJoiner;

/** Contains basic functions that are used often. */
public class Util {
  public static final double kEpsilon = 1e-12;

  /** Prevent this class from being instantiated. */
  private Util() {}

  /** Limits the given input to the given magnitude. */
  public static double limit(double v, double maxMagnitude) {
    return limit(v, -maxMagnitude, maxMagnitude);
  }

  public static double limit(double v, double min, double max) {
    return Math.min(max, Math.max(min, v));
  }

  public static boolean inRange(double v, double maxMagnitude) {
    return inRange(v, -maxMagnitude, maxMagnitude);
  }

  /** Checks if the given input is within the range (min, max), both exclusive. */
  public static boolean inRange(double v, double min, double max) {
    return v > min && v < max;
  }

  public static double interpolate(double a, double b, double x) {
    x = limit(x, 0.0, 1.0);
    return a + (b - a) * x;
  }

  public static String joinStrings(final String delim, final List<?> strings) {
    StringBuilder sb = new StringBuilder();
    for (int i = 0; i < strings.size(); ++i) {
      sb.append(strings.get(i).toString());
      if (i < strings.size() - 1) {
        sb.append(delim);
      }
    }
    return sb.toString();
  }

  public static boolean epsilonEquals(double a, double b, double epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }

  public static boolean epsilonEquals(double a, double b) {
    return epsilonEquals(a, b, kEpsilon);
  }

  public static boolean epsilonEquals(int a, int b, int epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }

  public static boolean allCloseTo(final List<Double> list, double value, double epsilon) {
    boolean result = true;
    for (Double value_in : list) {
      result &= epsilonEquals(value_in, value, epsilon);
    }
    return result;
  }

  public static double handleDeadband(double value, double deadband) {
    deadband = Math.abs(deadband);
    if (deadband == 1) {
      return 0;
    }
    double scaledValue = (value + (value < 0 ? deadband : -deadband)) / (1 - deadband);
    return (Math.abs(value) > Math.abs(deadband)) ? scaledValue : 0;
  }

  public static double map(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

  public static String arrayToString(Object[] array, String delimeter) {
    StringJoiner joiner = new StringJoiner(delimeter);
    for (Object element : array) {
      joiner.add(element.toString());
    }
    return joiner.toString();
  }

  // Function to know if the alliecne color is blue
  public static boolean isBlueAllience() {
    if (!DriverStation.getAlliance().isPresent()) {
      DriverStation.reportWarning("Allience color not preset !!!", false);
      return false;
    }
    return DriverStation.getAlliance().get().toString() == DriverStation.Alliance.Blue.toString();
  }

  // Function to know if the alliecne color is red
  public static boolean isRedAllience() {
    if (!DriverStation.getAlliance().isPresent()) {
      DriverStation.reportWarning("Allience color not preset !!!", false);
      return false;
    }
    return DriverStation.getAlliance().get().toString() == DriverStation.Alliance.Red.toString();
  }

  // Funcion para obtener el delta mas corto entre dos angulos
  // Trabaja con angulos en el rango de 0-360
  // @param {double} double _target: angulo deseado
  // @param {double} double _actual: angulo actual
  public static double DeltaAngle(double _target, double _actual) {
    double deltadegrees = (_target - _actual);
    deltadegrees = ((deltadegrees + 180) - (Math.floor((deltadegrees + 180) / 360f) * 360)) - 180;
    return deltadegrees;
  }

  // Funcion que limita el angulo de 0 a 360
  // @param {double} double_angle: angulo que deseas limitar
  public static double Limit360Angle(double _angle) {
    _angle = _angle % 360;
    if (_angle < 0) {
      _angle += 360;
    }
    return _angle;
  }

  // Función para calcular el angulo entre 2 posiciones
  public static double calculateAngleBetweenPositions(Pose2d source, Pose2d target) {
    double angle =
        Math.atan2(
            target.getTranslation().getY() - source.getTranslation().getY(),
            target.getTranslation().getX() - source.getTranslation().getX());
    return angle;
  }

  // Funcion para aplicar rotaciones en z (con una matriz de rotacion y angulo en RADIANES)
  public static Pose3d applyYawRotation(Pose3d _target, double rotation) {
    double cosYaw = Math.cos(rotation);
    double sinYaw = Math.sin(rotation);
    double newX =
        _target.getTranslation().getX() * cosYaw - _target.getTranslation().getY() * sinYaw;
    double newY =
        _target.getTranslation().getX() * sinYaw + _target.getTranslation().getY() * cosYaw;
    return new Pose3d(
        new Translation3d(newX, newY, _target.getTranslation().getZ()), _target.getRotation());
  }
}
