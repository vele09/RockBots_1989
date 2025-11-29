package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;


public class FieldConstants {

    // Posiciones en la cancha azul
     public static Pose3d[] tagsmatrix =
      new Pose3d[] {
        new Pose3d(
            new Translation3d(16.697198, 0.65532, 1.4859),
            new Rotation3d(0, 0, Units.degreesToRadians(126))), // 1
        new Pose3d(
            new Translation3d(16.697198, 7.39648, 1.4859),
            new Rotation3d(0, 0, Units.degreesToRadians(234))), // 2
        new Pose3d(
            new Translation3d(11.56081, 8.05561, 1.30175),
            new Rotation3d(0, 0, Units.degreesToRadians(270))), // 3
        new Pose3d(
            new Translation3d(9.27608, 6.137656, 1.867916),
            new Rotation3d(0, Units.degreesToRadians(30), 0)), // 4
        new Pose3d(
            new Translation3d(9.27608, 1.914906, 1.867916),
            new Rotation3d(0, Units.degreesToRadians(30), 0)), // 5
        new Pose3d(
            new Translation3d(13.474446, 3.306318, 0.308102),
            new Rotation3d(0, 0, Units.degreesToRadians(300))), // 6
        new Pose3d(
            new Translation3d(13.890498, 4.0259, 0.308102),
            new Rotation3d(0, 0, Units.degreesToRadians(1800))), // 7
        new Pose3d(
            new Translation3d(13.474446, 4.745482, 0.308102),
            new Rotation3d(0, 0, Units.degreesToRadians(60))), // 8
        new Pose3d(
            new Translation3d(12.643358, 4.745482, 0.308102),
            new Rotation3d(0, 0, Units.degreesToRadians(120))), // 9
        new Pose3d(
            new Translation3d(12.227306, 4.0259, 0.308102),
            new Rotation3d(0, 0, Units.degreesToRadians(180))), // 10
        new Pose3d(
            new Translation3d(12.643358, 3.306318, 0.308102),
            new Rotation3d(0, 0, Units.degreesToRadians(240))), // 11
        new Pose3d(
            new Translation3d(0.851154, 0.65532, 1.4859),
            new Rotation3d(0, 0, Units.degreesToRadians(54))), // 12
        new Pose3d(
            new Translation3d(0.851154, 7.39648, 1.4859),
            new Rotation3d(0, 0, Units.degreesToRadians(306))), // 13
        new Pose3d(
            new Translation3d(8.272272, 6.137656, 1.613916),
            new Rotation3d(0, Units.degreesToRadians(30), Units.degreesToRadians(180))), // 14
        new Pose3d(
            new Translation3d(8.272272, 1.914906, 1.613916),
            new Rotation3d(0, Units.degreesToRadians(30), Units.degreesToRadians(180))), // 15
        new Pose3d(
            new Translation3d(5.987542, -0.00381, 1.30175),
            new Rotation3d(0, 0, Units.degreesToRadians(90))), // 16
        new Pose3d(
            new Translation3d(4.073906, 3.306318, 0.308102),
            new Rotation3d(0, 0, Units.degreesToRadians(240))), // 17
        new Pose3d(
            new Translation3d(3.6576, 4.0259, 0.308102),
            new Rotation3d(0, 0, Units.degreesToRadians(180))), // 18
        new Pose3d(
            new Translation3d(4.073906, 4.745482, 0.308102),
            new Rotation3d(0, 0, Units.degreesToRadians(120))), // 19
        new Pose3d(
            new Translation3d(4.90474, 4.745482, 0.308102),
            new Rotation3d(0, 0, Units.degreesToRadians(60))), // 20
        new Pose3d(
            new Translation3d(5.321046, 4.0259, 0.308102),
            new Rotation3d(0, 0, 0)), // 21
        new Pose3d(
            new Translation3d(4.90474, 3.306318, 0.308102),
            new Rotation3d(0, 0, Units.degreesToRadians(300))), // 22
      };
}