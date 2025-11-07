package frc.robot.subsystems.drive;

///////////////////////////////////////////////////////////////////////////////
// Description: Clase para administrar la logica del pid drive el el swerve (para alinaer contra los april tags).
// Notes:
//  - Esta clase esta disenada para solo regresar un comando que correremos, tener cuidado con los estados
//    globales que no se actualizan en el comando, porque estos se comparten.
///////////////////////////////////////////////////////////////////////////////

import lib.util.Util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DrivePidDrive {

    // Clase que representa una lectura de la camara que usamos de input para actualizar las mediciones
    public static class VisionMeasure {
        public int tagID = 0;
        public int cameraID = 0;
        public double timestamp=0;
        private Pose3d targetPoseCameraSpace = null;
        private Pose3d targetPoseFieldSpace = null;

        public VisionMeasure(int tagID, int cameraID, double timestamp,Pose3d targetPoseCameraSpace,Pose3d targetPoseFieldSpace){
            this.tagID=tagID;
            this.cameraID=cameraID;
            this.timestamp=timestamp;
            this.targetPoseCameraSpace=targetPoseCameraSpace;
            this.targetPoseFieldSpace=targetPoseFieldSpace;
        }
    }
    
    // Variable con la referencia al subsistema de drive
    private DriveSubsystem drive;
    // Last measure updated from the vision subsystem
    private VisionMeasure measure = null;
    // Variable to store last x error
    private double lastXError = 0;
    // Variable to store last y error
    private double lastYError = 0;
    // Variables to store last registred times
    private double[] lastRegisteredTime = null;
    // Variable to store the state of the pid drive
    private int state = 0;
    // Variable to store the cylces that fail to read value
    private int nullCyclesCounter = 0;
    // Variable to store the aligning zone
    private int zone = 0;

    // Class constructor
    public DrivePidDrive(DriveSubsystem drive){
        this.drive=drive;
        lastRegisteredTime = new double[]{0,0};
    }

    // Funcion para regresar el comando que corre el pid drive
    public Command runPidDrive(double setPointX, double setPointY, int zone){
        Command command = new SequentialCommandGroup(
            Commands.runOnce(() -> { startDrive(zone); },this.drive),
            Commands.run(() -> { runDrive(setPointX,setPointY); },this.drive)
        ).finallyDo((interrupt)->{ setState(0); drive.stopDrive(); });
        command.addRequirements(this.drive);
        return command;
    }

    // Funcion para inicializar el pid drive
    private void startDrive(int zone){
        // Update zone
        this.zone = zone;
        // Reset all the variables
        measure = null;
        lastXError = 100; // Set a high error to avoid end condition triggers before reading first value
        lastYError = 100;
        lastRegisteredTime = new double[]{0,0};
        nullCyclesCounter=0;
        setState(1);
    }

    // Logica principal del pid drive
    private void runDrive(double setPointX, double setPointY){
        // Verificamos la data este completa
        if(measure==null || measure.targetPoseCameraSpace==null || measure.targetPoseFieldSpace==null){
            stopByNullCycles();
            return;
        }
        // Aseguramos venga de una camara valida
        if(measure.cameraID!=1 && measure.cameraID!=2){
            stopByNullCycles();
            return;
        }
        // Move the target position in camera space to the robot space reference
        Pose3d targetRobotSpace = measure.targetPoseCameraSpace;
        // Quitamos los pose vacios y los que vengan de un tag invalido
        if(targetRobotSpace.getTranslation().getNorm()<0.3 || !isTagValid(measure.tagID)){
            stopByNullCycles();
            return;
        }
        // Aseguramos sea una medicion nueva
        if(lastRegisteredTime[measure.cameraID-1]==measure.timestamp){
            stopByNullCycles();
            return;
        }
        // Reset null counter cycle since we get a vlaid measure
        nullCyclesCounter=0;
        lastYError = targetRobotSpace.getX()-setPointY;
        lastXError = targetRobotSpace.getY()-setPointX;
        // System.out.println(measure.tagID);
        // System.out.println(targetRobotSpace.toString());
        // System.out.println("Distance Y: "+String.valueOf(targetRobotSpace.getX())+" Error X: "+String.valueOf(lastYError));
        // System.out.println("Distance X: "+String.valueOf(targetRobotSpace.getY())+" Error Y: "+String.valueOf(lastXError));
        if(state==1){
            // Fix robot angle setpoint to desire position aligned with tag
            drive.updateYawSetpoint(Math.toDegrees(measure.targetPoseFieldSpace.getRotation().getZ())+180);
            setState(2);
        }
        // Apply P controller
        double yInput = Util.limit((Math.abs(lastYError) > 0.5) ? lastYError * 0.4 : 
                                    (Math.abs(lastYError) > 0.2) ? lastYError * 0.5 : 
                                    (Math.abs(lastYError) > 0.1) ? lastYError * 0.7 : lastYError * 0.85 ,-1,1);
        double xInput = Util.limit((Math.abs(lastXError) > 0.5) ? lastXError * 0.4 : 
                                    (Math.abs(lastXError) > 0.2) ? lastXError * 0.5 :
                                    (Math.abs(lastXError) > 0.1) ? lastXError * 0.7 : lastXError * 0.85,-1,1);
        // Rotate the input values (since inputs are in robot space and we rotate to be field space)
        Pose3d tempInputs = new Pose3d(new Translation3d(xInput,yInput,0), new Rotation3d());
        tempInputs = Util.applyYawRotation(tempInputs,Math.toRadians(drive.getInputs().yawAngle*-1));
        drive.swerveMainDrive(tempInputs.getX(),tempInputs.getY(),0); //in drive x value moves left and right and y up and down
        // Update last measure registred time
        lastRegisteredTime[measure.cameraID-1]=measure.timestamp;
    }

    // Funcion para deneter el movimeinto en caso de mucho tiempo sin registrar medidas
    private void stopByNullCycles(){
        nullCyclesCounter++;
        if(nullCyclesCounter>10){ //seguridad por si deja de ver el tag
            drive.stopDrive();
        }
    }

    // Funcion para actualizar una medida con las camaras
    public void updateMeasure(VisionMeasure measure){
        this.measure = measure;
    }

    // Funcion para saber si ya termino de correr el pid
    public boolean finished(){
        return ((Math.abs(lastXError) < 0.012 && Math.abs(lastYError) < 0.012) &&
                Math.abs(drive.getYawAngleError())<0.75 && getState()!=0) ||
                (nullCyclesCounter>15 && getState()!=0);
    }

    // Funcion para saber si ya termino de correr el pid
    public boolean finishedCorrectly(){
        return ((Math.abs(lastXError) < 0.012 && Math.abs(lastYError) < 0.012) &&
                Math.abs(drive.getYawAngleError())<0.75);
    }

    // Funcion para leer el estado del pid drive
    public int getState(){
        return state;
    }

    // Funcion para setear el estado del pid drive
    private void setState(int state){
        this.state=state;
    }

    // Funcion para saber si es un tag valido dependiendo la zona que estamos ajustando
    public boolean isTagValid(int tagID){
        if(Util.isBlueAllience()){
            if(tagID==20 && zone == 0){
                return true;
            }else if(tagID==19 && zone == 1){
                return true;
            }else if(tagID==18 && zone == 2){
                return true;
            }else if(tagID==17 && zone == 3){
                return true;
            }else if(tagID==22 && zone == 4){
                return true;
            }else if(tagID==21 && zone == 5){
                return true;
            }
        }else{
            if(tagID==8 && zone == 0){
                return true;
            }else if(tagID==9 && zone == 1){
                return true;
            }else if(tagID==10 && zone == 2){
                return true;
            }else if(tagID==11 && zone == 3){
                return true;
            }else if(tagID==6 && zone == 4){
                return true;
            }else if(tagID==7 && zone == 5){
                return true;
            }
        }
        return false;
    }

}