package frc.robot.subsystems.vision;

///////////////////////////////////////////////////////////////////////////////
// Description: Clase para guardar la informacion de los filudials y sea serializable para usar como input e advantage kit
// Authors: Paola, Pablo
// Notes: - 
///////////////////////////////////////////////////////////////////////////////

import java.nio.ByteBuffer;
import lib.limelight.LimelightHelpers;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

public class FiducialObservation implements StructSerializable {
    public static class FiducialObservationStruct implements Struct<FiducialObservation> {
        public int id;
        public double txnc;
        public double tync;
        public double ambiguity;
        public double distToRobot;

        @Override
        public Class<FiducialObservation> getTypeClass() {
            return FiducialObservation.class;
        }

        @Override
        public String getTypeString() {
            return "struct:FiducialObservation";
        }

        @Override
        public String getTypeName() {
            return "struct:FiducialObservation";
        }

        @Override
        public int getSize() {
            return kSizeInt32 + 4 * kSizeDouble;
        }

        @Override
        public String getSchema() {
            return "int id;double txnc;double tync;double ambiguity;double distToRobot";
        }

        @Override
        public FiducialObservation unpack(ByteBuffer bb) {
            FiducialObservation rv = new FiducialObservation();
            rv.id = bb.getInt();
            rv.txnc = bb.getDouble();
            rv.tync = bb.getDouble();
            rv.ambiguity = bb.getDouble();
            rv.distToRobot = bb.getDouble();
            return rv;
        }

        @Override
        public void pack(ByteBuffer bb, FiducialObservation value) {
            bb.putInt(value.id);
            bb.putDouble(value.txnc);
            bb.putDouble(value.tync);
            bb.putDouble(value.ambiguity);
            bb.putDouble(value.distToRobot);
        }
    }

    public int id;
    public double txnc;
    public double tync;
    public double ambiguity;
    public double distToRobot;

    public FiducialObservation() {
    }

    public FiducialObservation(int id, double txnc, double tync, double ambiguity, double distToRobot) {
        this.id = id;
        this.txnc = txnc;
        this.tync = tync;
        this.ambiguity = ambiguity;
        this.distToRobot = distToRobot;
    }

    public static FiducialObservation fromLimelight(LimelightHelpers.RawFiducial fiducial) {
        FiducialObservation rv = new FiducialObservation();
        rv.id = fiducial.id;
        rv.txnc = fiducial.txnc;
        rv.tync = fiducial.tync;
        rv.ambiguity = fiducial.ambiguity;
        rv.distToRobot = fiducial.distToRobot;

        return rv;
    }

    public static FiducialObservation[] fromLimelight(LimelightHelpers.RawFiducial[] fiducials) {
        FiducialObservation[] rv = new FiducialObservation[fiducials.length];
        for (int i = 0; i < fiducials.length; ++i) {
            rv[i] = fromLimelight(fiducials[i]);
        }
        return rv;
    }

    public static final FiducialObservationStruct struct = new FiducialObservationStruct();
}