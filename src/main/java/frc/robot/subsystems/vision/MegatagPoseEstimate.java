package frc.robot.subsystems.vision;

///////////////////////////////////////////////////////////////////////////////
// Description: Clase para guardar la informacion de megatag y sea serializable para usar como input e advantage kit
// Authors: Paola, Pablo
// Notes: - 
///////////////////////////////////////////////////////////////////////////////

import java.nio.ByteBuffer;
import lib.limelight.LimelightHelpers;
import lib.util.MathHelpers;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

public class MegatagPoseEstimate implements StructSerializable {
    public static class MegatagPoseEstimateStruct implements Struct<MegatagPoseEstimate> {
        public Pose2d fieldToRobot = MathHelpers.kPose2dZero;
        public double timestampSeconds;
        public double latency;
        public double avgTagArea;

        @Override
        public Class<MegatagPoseEstimate> getTypeClass() {
            return MegatagPoseEstimate.class;
        }

        @Override
        public String getTypeString() {
            return "struct:MegatagPoseEstimate";
        }

        @Override
        public String getTypeName() {
            return "struct:MegatagPoseEstimate";
        }

        @Override
        public int getSize() {
            return Pose2d.struct.getSize() + kSizeDouble * 3;
        }

        @Override
        public String getSchema() {
            return "Pose2d fieldToRobot;double timestampSeconds;double latency;double avgTagArea";
        }

        @Override
        public Struct<?>[] getNested() {
            return new Struct<?>[] { Pose2d.struct };
        }

        @Override
        public MegatagPoseEstimate unpack(ByteBuffer bb) {
            MegatagPoseEstimate rv = new MegatagPoseEstimate();
            rv.fieldToRobot = Pose2d.struct.unpack(bb);
            rv.timestampSeconds = bb.getDouble();
            rv.latency = bb.getDouble();
            rv.avgTagArea = bb.getDouble();
            rv.fiducialIds = new int[0];
            return rv;
        }

        @Override
        public void pack(ByteBuffer bb, MegatagPoseEstimate value) {
            Pose2d.struct.pack(bb, value.fieldToRobot);
            bb.putDouble(value.timestampSeconds);
            bb.putDouble(value.latency);
            bb.putDouble(value.avgTagArea);
        }
    }

    public Pose2d fieldToRobot = MathHelpers.kPose2dZero;
    public double timestampSeconds;
    public double latency;
    public double avgTagArea;
    public int[] fiducialIds;

    public MegatagPoseEstimate() {
    }

    public static MegatagPoseEstimate fromLimelight(LimelightHelpers.PoseEstimate poseEstimate) {
        MegatagPoseEstimate rv = new MegatagPoseEstimate();
        rv.fieldToRobot = poseEstimate.pose;
        if (rv.fieldToRobot == null)
            rv.fieldToRobot = MathHelpers.kPose2dZero;
        rv.timestampSeconds = poseEstimate.timestampSeconds;
        rv.latency = poseEstimate.latency;
        rv.avgTagArea = poseEstimate.avgTagArea;
        rv.fiducialIds = new int[poseEstimate.rawFiducials.length];
        for (int i = 0; i < rv.fiducialIds.length; ++i) {
            rv.fiducialIds[i] = poseEstimate.rawFiducials[i].id;
        }

        return rv;
    }

    public static final MegatagPoseEstimateStruct struct = new MegatagPoseEstimateStruct();
}