package com.team3478.lib.kinematics;

import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

import com.team3478.frc2025.Constants;
import com.team3478.frc2025.RobotContainer;
import com.team3478.frc2025.subsystems.drive.DriveZone;
import com.team3478.frc2025.subsystems.elevator.ElevatorSubsystem.ElevatorPosition;
import com.team3478.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

// Class to manage the arm kinematics and util functions needed

public class KinematicArm {

    // Areglos para guardar los poses de todos los branches
    private Pose3d[] redReefBranches = new Pose3d[36];
    private Pose3d[] blueReefBranches = new Pose3d[36];

    private final RobotContainer container;
    private final DriveZone driveZone;

    // Class constructor
    public KinematicArm(RobotContainer container) {
        this.container=container;
        this.driveZone = new DriveZone();
        blueReefBranches = generateReefBranches(driveZone.getBlueReefCenter());
        redReefBranches = generateReefBranches(driveZone.getRedReefCenter());
    }

    /**
     * Generates an array of Pose3d objects representing the branches on the reef structure.
     * 
     * The reefBranches array follows a structured order:
     * - The REEF has 6 faces, each with 2 vertical pipes (left and right).
     * - Each pipe has 3 branches, positioned at different height levels (L2, L3, L4).
     * - The array is indexed as follows:
     *   - Face 1: Left L2, Right L2, Left L3, Right L3, Left L4, Right L4
     *   - Face 2: Left L2, Right L2, Left L3, Right L3, Left L4, Right L4
     *   - ...
     *   - Face 6: Left L2, Right L2, Left L3, Right L3, Left L4, Right L4
     * - This results in a total of 36 branches (6 faces × 2 pipes × 3 levels).
     * - To access a specific branch:
     *   - Index = (faceIndex * 6) + (levelIndex * 2) + pipeSide
     *     - faceIndex (0-5): The hexagon face index
     *     - levelIndex (0-2): The branch height level (L2, L3, L4)
     *     - pipeSide (0 = Left, 1 = Right): Left or right pipe on the face
     * 
     * - La cara 0 es la de la zona 5 
     */
    private Pose3d[] generateReefBranches(Translation2d reefCenter) {
        Pose3d[] reefBranches = new Pose3d[36];
        double radius = 0.75; // Estimated hexagon radius
        double pipeSpacing = 0.33 / 2.0; // Half-spacing for two pipes per face
        double[] faceAngles = {0, 60, 120, 180, 240, 300}; // Faces of hexagon (starting at X+)
        int index = 0;
        for (double faceAngle : faceAngles) {
            double faceRad = Math.toRadians(faceAngle);
            // Get tube center point on the hexagon face
            double tubeX = reefCenter.getX() + radius * Math.cos(faceRad); // X is forward
            double tubeY = reefCenter.getY() + radius * Math.sin(faceRad); // Y is left
            // Split left and right pipe positions along the face direction
            Translation2d leftPipe = new Translation2d(tubeX + pipeSpacing * Math.sin(faceRad),  // Y-based split
                                                       tubeY - pipeSpacing * Math.cos(faceRad)); // X-based split
            Translation2d rightPipe = new Translation2d(tubeX - pipeSpacing * Math.sin(faceRad),
                                                        tubeY + pipeSpacing * Math.cos(faceRad));
            // Define L2, L3, L4 heights
            double[] heights = {0.81, 1.22, 1.82};
            for (int level = 0; level < 3; level++) {
                double height = heights[level];
                reefBranches[index++] = new Pose3d(leftPipe.getX(), leftPipe.getY(), height, 
                                                   new Rotation3d());
                reefBranches[index++] = new Pose3d(rightPipe.getX(), rightPipe.getY(), height, 
                                                   new Rotation3d());
            }
        }
        return reefBranches;
    }    

    // Funcion para leer las posiciones de los reef del lado azul
    public Pose3d[] getBlueReefPoints(){
        return blueReefBranches;
    }

    // Funcion para leer las posiciones de los reef del lado rojo
    public Pose3d[] getRedReefPoints(){
        return redReefBranches;
    }

    // Funcion para regresar el endpoint a utilizar (punto 3d en la cancha donde queremos el end efector)
    public Pose3d getEndPoint(ElevatorPosition position){
        // Get current robot pose from odometry
        Pose2d robotPose = container.getDriveSubsystem().getPose();
        // Determine the robot's zone
        int zoneIndex = driveZone.getRobotZone(robotPose);
        if (zoneIndex == -1) {
            // The robot is not in a valid zone
            return null;
        }
        // Determine if the robot is aligned left or right of target
        Pose2d zonePoint = driveZone.getReefZonesPathPoints(zoneIndex);
        String side = getRelativePosition(zonePoint.getTranslation(),zonePoint.getRotation(),robotPose.getTranslation());
        if(side.equals("Aligned")){
            // Cannot determine which side of the reef is using
            return null;
        }
        // Fetch the target pose for this zone
        Pose3d endPoint = getBase3dPoint(position,zoneIndex,side);
        if(endPoint==null){
            // Fail to determine correct endpoint
            return null;
        }
        return endPoint;
    }

    // Funcion para seleccionar el punto 3d correcto de acuerdo a donde queremos alinear
    private Pose3d getBase3dPoint(ElevatorPosition position, int zoneIndex, String side){
        // Ajustamos la zona al orden de las caras
        zoneIndex = zoneIndex+1;
        if(zoneIndex>5) zoneIndex-=6;
        // Usamos el side para ajustar el offset
        int sideIndex = 0;
        if(side=="Right"){
            sideIndex=1;
        }
        // Usamos el level para ajustar el offset
        int level = 0;
        if(position==ElevatorPosition.L2CoralPosition){
            level=1;
        }else if(position==ElevatorPosition.L3CoralPosition){
            level=2;
        }
        // Calculamos el index de la matriz
        int index = zoneIndex*6 + level*2 + sideIndex;
        if(index<0 || index >=blueReefBranches.length){
            // Wrong index estimation
            return null;
        }
        if(Util.isBlueAllience()){
            return blueReefBranches[index];
        }else{
            return redReefBranches[index];
        }
    }

    // Function to determine if pointB is to the left or right of pointA
    private String getRelativePosition(Translation2d pointA, Rotation2d heading, Translation2d pointB) {
        // Compute unit direction vector from the heading (where the reference is facing)
        Translation2d direction = new Translation2d(heading.getCos(), heading.getSin());
        // Compute relative position vector from pointA to pointB
        Translation2d relative = new Translation2d(
            pointB.getX() - pointA.getX(),
            pointB.getY() - pointA.getY()
        );
        // Compute 2D cross product to determine which side pointB is on
        double side = direction.getX() * relative.getY() - direction.getY() * relative.getX();
        if (side > 0) {
            return "Left";
        } else if (side < 0) {
            return "Right";
        } else {
            return "Aligned";
        }
    }

    public Result calculateElevatorAndGripperAngle(
            Translation3d desiredPosition,
            Pose2d robotPose
    ) {
        // --------------------------------------------------
        // 1) Transform field coords (X,Y,Z) -> robot reference
        // --------------------------------------------------
        double dx = desiredPosition.getX() - robotPose.getX();
        double dy = desiredPosition.getY() - robotPose.getY();
        double thetaRobot = robotPose.getRotation().getRadians();

        // Undo robot yaw
        double xRobot =  dx * Math.cos(-thetaRobot) - dy * Math.sin(-thetaRobot);
        double zRobot =  desiredPosition.getZ();  // unaffected by yaw in typical 2D

        // --------------------------------------------------
        // 2) Setup / Constants
        // --------------------------------------------------
        double dX = Constants.MechanismKinematics.kDX; // pivot offset X
        double dZ = Constants.MechanismKinematics.kDZ; // pivot offset Z
        double L1 = Constants.MechanismKinematics.kL1;
        double L2 = Constants.MechanismKinematics.kL2;
        double offsetDeg = Constants.MechanismKinematics.kAngle;
        double offsetRad = Math.toRadians(offsetDeg);

        // Elevator constraints
        double hMin = Constants.Elevator.kElevatorLowerLimit;
        double hMax = Constants.Elevator.kElevatorUpperLimit;

        // Gripper angle constraints (in DEGREES in your constants, so convert to radians)
        double GripperMinRad = Math.toRadians(90) - Math.toRadians(Constants.Gripper.kArmForwardKinematicLimit); // ajustamos el 90 del mecanismo a la logica que considera los angulos contra la horizontal
        double GripperMaxRad = Math.toRadians(90) - Math.toRadians(Constants.Gripper.kArmReverseKinematicLimit);

        // --------------------------------------------------
        // 3) Local target from pivot’s perspective
        // --------------------------------------------------
        double xRelative = xRobot - dX;
        double zRelative = zRobot - dZ;

        // Quick out: if beyond total horizontal reach, no solution
        double maxReach = L1 + L2;
        if (Math.abs(xRelative) > maxReach) {
            //System.out.println("Unreachable X: out of horizontal range");
            return null;
        }

        // --------------------------------------------------
        // PART A: Try the standard 2-solution closed-form
        // --------------------------------------------------

        // 4) Solve eqn for horizontal: L1*cos(a) + L2*cos(a+offset) = xRel
        double A = L1 + L2 * Math.cos(offsetRad);
        double B = L2 * Math.sin(offsetRad);
        double R = Math.sqrt(A*A + B*B);
        double alpha = Math.atan2(B, A);

        double xRelOverR = xRelative / R;
        // If outside [-1, +1], no real solution for acos
        if (Math.abs(xRelOverR) > 1.0) {
            // We'll skip directly to fallback iteration below
            return tryIterativeFallback(xRelative, zRelative,
                                        L1, L2, offsetRad,
                                        GripperMinRad, GripperMaxRad,
                                        hMin, hMax);
        }

        double acosVal = Math.acos(xRelOverR); // in [0..pi]
        double a1 = (+acosVal) - alpha;
        double a2 = (-acosVal) - alpha;

        // 5) Solve eqn for elevator from vertical: h = zRel - [L1 sin(a) + L2 sin(a+offset)]
        double h1 = zRelative - (L1*Math.sin(a1) + L2*Math.sin(a1 + offsetRad));
        double h2 = zRelative - (L1*Math.sin(a2) + L2*Math.sin(a2 + offsetRad));

        // Build small struct to hold each solution, with validity checks
        class ArmSolution {
            double angleRad;
            double elevator;
            boolean valid;
            ArmSolution(double angleRad, double elevator) {
                this.angleRad = angleRad;
                this.elevator = elevator;
                boolean inElevRange = (elevator >= hMin && elevator <= hMax);
                boolean inAngleRange = (angleRad >= GripperMinRad && angleRad <= GripperMaxRad);
                this.valid = inElevRange && inAngleRange;
            }
        }

        ArmSolution sol1 = new ArmSolution(a1, h1);
        ArmSolution sol2 = new ArmSolution(a2, h2);

        // 6) Pick the "top" valid solution if any
        List<ArmSolution> solutions = Arrays.asList(sol1, sol2);
        Optional<ArmSolution> bestAnalytic = solutions.stream()
            .filter(s -> s.valid)
            .max(Comparator.comparingDouble(s -> s.elevator));

        if (bestAnalytic.isPresent()) {
            // We found a valid solution that satisfies elevator + angle constraints
            ArmSolution best = bestAnalytic.get();
            //System.out.println("Using direct solution. a="+Math.toDegrees(best.angleRad)+" h="+best.elevator);
            return new Result(best.elevator, Math.toDegrees(best.angleRad));
        }

        // If we reach here => both solutions invalid (angle or elevator out of range),
        // fallback to iteration:
        return tryIterativeFallback(xRelative, zRelative,
                                    L1, L2, offsetRad,
                                    GripperMinRad, GripperMaxRad,
                                    hMin, hMax);
    }

    /**
     * Fallback: we will iterate over angles in [GripperMinRad..GripperMaxRad],
     * compute elevator for each, check if in [hMin..hMax], and pick the one that
     * yields the smallest X error from xRelative. We'll match z exactly.
     */
    private Result tryIterativeFallback(
            double xRelative, 
            double zRelative,
            double L1, double L2,
            double offsetRad,
            double GripperMinRad, double GripperMaxRad,
            double hMin, double hMax
    ) {
        // We'll do a small step in DEGREES for the Gripper angle
        double stepDeg = 1.0;  // Smaller step => more accuracy, more CPU. Usually 1 deg is fine.
        double bestXError = Double.POSITIVE_INFINITY;
        double bestElev = 0;
        double bestAngleRad = 0;

        for (double angDeg = Math.toDegrees(GripperMinRad);
            angDeg <= Math.toDegrees(GripperMaxRad);
            angDeg += stepDeg)
        {
            double a = Math.toRadians(angDeg);

            double xArm = L1*Math.cos(a) + L2*Math.cos(a + offsetRad);
            double zArm = L1*Math.sin(a) + L2*Math.sin(a + offsetRad);

            // Elevator needed to exactly match the desired Z
            double elev = zRelative - zArm;

            if (elev < hMin || elev > hMax) {
                continue;  // skip if elevator out of range
            }
            // See how close we get in X
            double xError = Math.abs(xArm - xRelative);
            if (xError < bestXError) {
                bestXError = xError;
                bestElev = elev;
                bestAngleRad = a;
            }
        }

        // If bestXError is still INF => no angles were feasible
        if (Double.isInfinite(bestXError)) {
            //System.out.println("Unreachable fallback: no angles produce valid elevator");
            return null;
        }

        // Optionally enforce some maximum allowed X error if you truly want a "close" solution
        double tolerance = 0.05;  // 5 cm, for example
        if (bestXError > tolerance) {
            //System.out.println("Unreachable fallback: best X error = "+bestXError+" > "+tolerance);
            return null;
        }

        //System.out.println("Using fallback iteration. a="+Math.toDegrees(bestAngleRad)+" h="+bestElev+", xError="+bestXError);
        return new Result(bestElev, Math.toDegrees(bestAngleRad));
    }



    // Esta funcion funciona, pero no considera los constrains the angulo del Gripper
    // /**
    //  * Calculates the required elevator height and Gripper pitch angle to reach a desired field position,
    //  * considering an L-shaped arm where L1 is the first rotating segment, and L2 extends at an angle x from L1.
    //  *
    //  * @param desiredPosition The desired 3D position in the field frame.
    //  * @param robotPose The current pose of the robot.
    //  * @return A result containing the required elevator height and Gripper pitch angle, or null if unreachable.
    //  */
    // public Result calculateElevatorAndGripperAngle(
    //         Translation3d desiredPosition,
    //         Pose2d robotPose
    // ) {
    //     // ----------------------------------------------
    //     // 1) Transform (X,Y,Z) into the robot reference
    //     // ----------------------------------------------
    //     double dx = desiredPosition.getX() - robotPose.getX();
    //     double dy = desiredPosition.getY() - robotPose.getY();
    //     double thetaRobot = robotPose.getRotation().getRadians();

    //     // Undo the robot yaw
    //     double xRobot =  dx * Math.cos(-thetaRobot) - dy * Math.sin(-thetaRobot);
    //     double zRobot =  desiredPosition.getZ();  // Z unaffected by yaw in typical 2D approximation

    //     // ----------------------------------------------
    //     // 2) Mechanism constants & setup
    //     // ----------------------------------------------
    //     double dX = Constants.MechanismKinematics.kDX;  // Horizontal offset from robot origin to pivot
    //     double dZ = Constants.MechanismKinematics.kDZ;  // Initial elevator height offset from the floor
    //     double L1 = Constants.MechanismKinematics.kL1;  // First arm segment length
    //     double L2 = Constants.MechanismKinematics.kL2;  // Second arm segment length

    //     double hMin = Constants.Elevator.kElevatorLowerLimit;
    //     double hMax = Constants.Elevator.kElevatorUpperLimit;

    //     // Angle between the L1 and L2 links
    //     double offsetDeg = Constants.MechanismKinematics.kAngle;
    //     double offsetRad = Math.toRadians(offsetDeg);

    //     // ----------------------------------------------
    //     // 3) "Local" target from pivot’s point of view
    //     // ----------------------------------------------
    //     // pivot is horizontally at "dX", elevator height is "h" (unknown).
    //     // So horizontally, the end effector must be xRelative = (xRobot - dX).
    //     // vertically, the end effector is (zRelative) above the floor,
    //     // but pivot will be at height "h" above floor.
    //     double xRelative = xRobot - dX;
    //     double zRelative = zRobot - dZ;

    //     // Validamos si el Gripper puede alcanzar el punto que deseamos
    //     double maxReach = L1 + L2;  // maximum horizontal extension (ignoring small offset angles)
    //     if (Math.abs(xRelative) > maxReach) {
    //         System.out.println("Unreachable X: out of horizontal range");
    //         return null;
    //     }

    //     // ----------------------------------------------
    //     // 4) Solve for pivot angle(s) from eqn (1):
    //     //
    //     //     L1*cos(a) + L2*cos(a + offset) = xRel
    //     //
    //     // We'll define:
    //     //     A = L1 + L2*cos(offset)
    //     //     B = L2*sin(offset)
    //     // so that eqn becomes:
    //     //     A*cos(a) - B*sin(a) = xRel
    //     //
    //     // which can be written as:  R*cos(a + alpha) = xRel
    //     // where R = sqrt(A^2 + B^2), alpha = atan2(B, A).
    //     // => a + alpha = ± acos( xRel / R ), leading to up to 2 solutions for a.
    //     // ----------------------------------------------
    //     double A = L1 + L2 * Math.cos(offsetRad);
    //     double B = L2 * Math.sin(offsetRad);

    //     double R = Math.sqrt(A*A + B*B);
    //     double alpha = Math.atan2(B, A);  // phase offset

    //     double xRelOverR = xRelative / R;
    //     // If |xRelOverR| > 1, no real solutions for acos.
    //     if (Math.abs(xRelOverR) > 1.0) {
    //         System.out.println("Unreachable X: no real pivot angles for given target");
    //         return null;
    //     }

    //     // primary angle from the arccos
    //     double acosVal = Math.acos(xRelOverR);  // in [0, π]

    //     // Two possible solutions for (a + alpha):
    //     //    sol1: a + alpha = +acosVal
    //     //    sol2: a + alpha = -acosVal
    //     // so:
    //     double a1 =  (+acosVal) - alpha;
    //     double a2 =  (-acosVal) - alpha;

    //     // ----------------------------------------------
    //     // 5) For each pivot angle, compute elevator height h from eqn (2):
    //     //    h = zRel - [ L1 sin(a) + L2 sin(a+offset ) ]
    //     // ----------------------------------------------
    //     double h1 = zRelative - ( L1 * Math.sin(a1) + L2 * Math.sin(a1 + offsetRad) );
    //     double h2 = zRelative - ( L1 * Math.sin(a2) + L2 * Math.sin(a2 + offsetRad) );

    //     // We'll keep track of each solution in a small helper object:
    //     class ArmSolution {
    //         double pitchRad;
    //         double elevator;
    //         boolean valid;

    //         ArmSolution(double pitchRad, double elevator) {
    //             this.pitchRad = pitchRad;
    //             this.elevator = elevator;
    //             // Check elevator constraints:
    //             this.valid = (elevator >= hMin && elevator <= hMax);
    //         }
    //     }

    //     ArmSolution sol1 = new ArmSolution(a1, h1);
    //     ArmSolution sol2 = new ArmSolution(a2, h2);

    //     // ----------------------------------------------
    //     // 6) We want to "prioritize the solution that keeps the elevator higher".
    //     //    So let's sort them by elevator height descending, and pick the first valid.
    //     // ----------------------------------------------
    //     List<ArmSolution> solutions = Arrays.asList(sol1, sol2);
    //     Optional<ArmSolution> bestSolution = solutions.stream()
    //         .filter(s -> s.valid)
    //         .max(Comparator.comparingDouble(s -> s.elevator));

    //     if (bestSolution.isPresent()) {
    //         return new Result(bestSolution.get().elevator, Math.toDegrees(bestSolution.get().pitchRad));
    //     }

    //     // If neither is valid:
    //     System.out.println("Unreachable: Both solutions out of elevator range.");
    //     return null;
    // }

    /**
     * Computes the field position (X, Z) of the end effector given an elevator height and mechanism angle.
     *
     * @param elevatorHeight The height of the elevator relative to its lowest position.
     * @param pitchAngle The pitch angle of the mechanism (0° = vertical, positive = right, negative = left).
     * @param robotPose The current pose of the robot.
     * @return The field position (X, Z) of the end effector.
     */
    public Translation3d getEndEffectorPosition(double elevatorHeight, double pitchAngle, Pose2d robotPose) {
        // ---------------------------------------------------
        // 1) Retrieve Mechanism Constants
        // ---------------------------------------------------
        double dX = Constants.MechanismKinematics.kDX;  // Pivot point horizontal offset
        double dZ = Constants.MechanismKinematics.kDZ;  // Pivot point height above floor
        double L1 = Constants.MechanismKinematics.kL1;  // First arm segment length
        double L2 = Constants.MechanismKinematics.kL2;  // Second arm segment length
        double offsetDeg = Constants.MechanismKinematics.kAngle; // L2 offset from L1
        double offsetRad = Math.toRadians(offsetDeg);

        // Convert pitch angle from mechanism reference to standard trigonometric angle
        double a = Math.toRadians(90) - Math.toRadians(pitchAngle); // Adjusted to match original inverse kinematics

        // ---------------------------------------------------
        // 2) Compute Pivot Position
        // ---------------------------------------------------
        double pivotX = robotPose.getX() + dX;  // Pivot is offset from robot
        double pivotZ = dZ + elevatorHeight;   // Pivot is raised by elevator

        // ---------------------------------------------------
        // 3) Compute End Effector Position
        // ---------------------------------------------------
        double effectorX = pivotX + (L1 * Math.cos(a)) + (L2 * Math.cos(a + offsetRad));
        double effectorZ = pivotZ + (L1 * Math.sin(a)) + (L2 * Math.sin(a + offsetRad));

        return new Translation3d(effectorX, robotPose.getY(), effectorZ); // Y remains the same
    }


    // Helper class to store the result
    public static class Result {
        public final double elevatorHeight;
        public final double GripperPitchAngle;

        public Result(double elevatorHeight, double GripperPitchAngle) {
            this.elevatorHeight = elevatorHeight;
            this.GripperPitchAngle = GripperPitchAngle;
        }

        @Override
        public String toString() {
            return "Elevator Height: " + elevatorHeight + " m, Gripper Pitch Angle: " + GripperPitchAngle + " degrees";
        }
    }
}
