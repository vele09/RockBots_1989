package frc.robot.subsystems.drive;

///////////////////////////////////////////////////////////////////////////////
// Description: Clase para generar zonas alrededor del reef y nos permita saber a cual autoalinearnos.
// Notes:
//  - The zone 0 is always the top right side of the hexagon.
//  - La logica considera que el 0,0 es en el lado azul y no se cambia este origen cuando se es rojo
///////////////////////////////////////////////////////////////////////////////

import java.util.ArrayList;
import java.util.List;
import lib.util.Util;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class DriveZone {

    // Vraiables to store the triangle zones limits
    private List<Translation2d[]> reefBlueZones = null;
    private List<Translation2d[]> reefRedZones = null;
    private Translation2d[] hexagonCenters = null;

    // Puntos a los que el path se va mover dependiendo la zona en el hexágono azul
    private Pose2d[] reefZonesBluePathPoints = new Pose2d[] {
        // Tag #20 (original yaw = 60, approach angle = -120)
        new Pose2d(
            new Translation2d(5.2297, 5.3094),
            Rotation2d.fromDegrees(-120)
        ),
        // Tag #19 (yaw = 120, approach angle = -60)
        new Pose2d(
            new Translation2d(3.7489, 5.3094),
            Rotation2d.fromDegrees(-60)
        ),
        // Tag #18 (yaw = 180, approach angle = 0)
        new Pose2d(
            new Translation2d(3.0076, 4.0259),
            Rotation2d.fromDegrees(0)
        ),
        // Tag #17 (yaw = 240, approach angle = 60)
        new Pose2d(
            new Translation2d(3.7489, 2.7424),
            Rotation2d.fromDegrees(60)
        ),
        // Tag #22 (yaw = 300, approach angle = 120)
        new Pose2d(
            new Translation2d(5.2297, 2.7424),
            Rotation2d.fromDegrees(120)
        ),
        // Tag #21 (yaw = 0, approach angle = 180)
        new Pose2d(
            new Translation2d(5.9710, 4.0259),
            Rotation2d.fromDegrees(180)
        )
    };

    private Pose2d[] reefZonesRedPathPoints = new Pose2d[] {
        // Tag #8  (yaw = 60, approach angle = -120)
        new Pose2d(
            new Translation2d(13.7994, 5.3094),
            Rotation2d.fromDegrees(-120)
        ),
        // Tag #9  (yaw = 120, approach angle = -60)
        new Pose2d(
            new Translation2d(12.3184, 5.3094),
            Rotation2d.fromDegrees(-60)
        ),
        // Tag #10 (yaw = 180, approach angle = 0)
        new Pose2d(
            new Translation2d(11.5773, 4.0259),
            Rotation2d.fromDegrees(0)
        ),
        // Tag #11 (yaw = 240, approach angle = 60)
        new Pose2d(
            new Translation2d(12.3184, 2.7424),
            Rotation2d.fromDegrees(60)
        ),
        // Tag #6  (yaw = 300, approach angle = 120)
        new Pose2d(
            new Translation2d(13.7994, 2.7424),
            Rotation2d.fromDegrees(120)
        ),
        // Tag #7  (yaw = 0,   approach angle = 180)
        new Pose2d(
            new Translation2d(14.5405, 4.0259),
            Rotation2d.fromDegrees(180)
        )
    };

    // Class constructor
    public DriveZone(){
        hexagonCenters = new Translation2d[]{
            new Translation2d(4.4893,4.0258), // blue
            new Translation2d(13.0607, 4.0258), // red
        };
        reefBlueZones = generateHexagonTriangles(new Pose2d(hexagonCenters[0],new Rotation2d()),5);
        reefRedZones = generateHexagonTriangles(new Pose2d(hexagonCenters[1],new Rotation2d()),5);
    }

    // Funcion para generar los triangulos de las zonas sobre el hexagono de la cancha
    private List<Translation2d[]> generateHexagonTriangles(Pose2d centerPose, double sideLength) {
        List<Translation2d[]> triangles = new ArrayList<>();
        // Center of the hexagon
        Translation2d center = centerPose.getTranslation();
        // Apply a 60° offset to align the first triangle correctly
        double angleOffset = Math.toRadians(30);
    
        // Generate the 6 vertices of the hexagon
        Translation2d[] vertices = new Translation2d[6];
        for (int i = 0; i < 6; i++) {
            double angle = Math.toRadians(60 * i) + angleOffset;
            Pose3d temp = new Pose3d(new Translation3d(sideLength,0,0),new Rotation3d());
            temp = Util.applyYawRotation(temp, angle);
            vertices[i] = new Translation2d(center.getX()+temp.getX(),center.getY()+temp.getY());
        }
    
        // Create triangles using the center and adjacent vertices
        for (int i = 0; i < 6; i++) {
            Translation2d[] triangle = new Translation2d[3];
            triangle[0] = center;                   // Center point
            triangle[1] = vertices[i];              // Current vertex
            triangle[2] = vertices[(i + 1) % 6];    // Next vertex (wrap around with modulus)
            triangles.add(triangle);
        }
    
        return triangles;
    }    

    // Funcion para hacer el calculo matematico y saber si un pose2d esta dentro de una de las zonas triangulares
    private boolean isPoseInsideTriangle(Pose2d pose, Translation2d[] triangle) {
        if (triangle.length != 3) {
            throw new IllegalArgumentException("Triangle must have exactly 3 vertices.");
        }
        Translation2d p = pose.getTranslation();
        Translation2d a = triangle[0];
        Translation2d b = triangle[1];
        Translation2d c = triangle[2];
        // Vectors from A
        double v0x = c.getX() - a.getX();
        double v0y = c.getY() - a.getY();
        double v1x = b.getX() - a.getX();
        double v1y = b.getY() - a.getY();
        double v2x = p.getX() - a.getX();
        double v2y = p.getY() - a.getY();
        // Dot products
        double dot00 = v0x * v0x + v0y * v0y;
        double dot01 = v0x * v1x + v0y * v1y;
        double dot02 = v0x * v2x + v0y * v2y;
        double dot11 = v1x * v1x + v1y * v1y;
        double dot12 = v1x * v2x + v1y * v2y;
        // Barycentric coordinates
        double denom = (dot00 * dot11 - dot01 * dot01);
        if (denom == 0) {
            return false; // Degenerate triangle
        }
        double invDenom = 1.0 / denom;
        double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
        double v = (dot00 * dot12 - dot01 * dot02) * invDenom;
        // Check if point is inside the triangle
        return (u >= 0) && (v >= 0) && (u + v <= 1);
    }

    // Funcion para saber si el robot esta en alguna zona
    public int getRobotZone(Pose2d robotPose){
        int index=0;
        if(Util.isBlueAllience()){
            for(var triangle: reefBlueZones){
                if(isPoseInsideTriangle(robotPose,triangle)){
                    return index;
                }
                index++;
            }
        }else{
            for(var triangle: reefRedZones){
                if(isPoseInsideTriangle(robotPose,triangle)){
                    return index;
                }
                index++;
            }
        }
        return -1;
    }

    // Funcion para regresar el punto (a donde debe alinearse el robot) en base a la zona seleccionada
    public Pose2d getReefZonesPathPoints(int index){
        if(Util.isBlueAllience()){
            return reefZonesBluePathPoints[index];
        }else{
            return reefZonesRedPathPoints[index];
        }
    }

    // Funcion para regresar el centro del reef rojo (del hexagono)
    public Translation2d getRedReefCenter(){
        return hexagonCenters[1];
    }

    // Funcion para regresar el centro del reef azul (del hexagono)
    public Translation2d getBlueReefCenter(){
        return hexagonCenters[0];
    }
    
}
