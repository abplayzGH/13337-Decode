package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

public class LimeLight {
    // Dont forget to run Limelight Hardware Manager
    // IP Address http://172.28.0.1:5801/
    public Limelight3A vision;
    private LLResult result;

    //Store the information of the april tag we are tracking
    private int tagID;
    private double tagLocationX, tagLocationY, tagDistance;
    private double tagLocationArea, tagLocationWidth, tagLocationHeight;
    private double angleToGoal; //in radians
    public Pose3D tagPose = new Pose3D(
            new Position(DistanceUnit.INCH,0,0,0,0),
            new YawPitchRollAngles(AngleUnit.DEGREES,0,0,0,0));
    public int getTagID()
    {
        return tagID;
    }

    public double getTagLocationX()
    {
        return tagLocationX;
    }
    public double getTagLocationY()
    {
        return tagLocationY;
    }
    public double getTagDistance()
    {
        return tagDistance;
    }
    public LimeLight(Limelight3A vision)
    {
        this.vision = vision;

        setPipeline(0);
        vision.start();
        clearFoundAprilTag();
    }

    public LimeLight(Limelight3A vision, int pipeline)
    {
        this.vision = vision;

        setPipeline(pipeline);
        vision.start();
        clearFoundAprilTag();
    }

    public void setPipeline(int pipeline) {vision.pipelineSwitch(pipeline);}

    public int getPipeline()
    {
        return result != null ? result.getPipelineIndex() : -1;
    }

    public void clearFoundAprilTag()
    {
        tagID = 0;
        tagLocationX = 0;
        tagLocationY = 0;
        tagLocationArea = 0;
        tagLocationWidth = 0;
        tagLocationHeight = 0;
        tagDistance = 0;
    }

    public boolean hasValidTarget(){
        return result != null && result.isValid() && !result.getFiducialResults().isEmpty();
    }

    public void getAprilTags()
    {
        result = vision.getLatestResult();
        if (result == null) { clearFoundAprilTag(); return; }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            tagID = fiducial.getFiducialId();
            tagLocationX = fiducial.getTargetXDegrees();
            tagLocationY = fiducial.getTargetYDegrees();
            tagLocationWidth = fiducial.getTargetXPixels();
            tagLocationHeight = fiducial.getTargetYPixels();
            tagLocationArea = tagLocationWidth * tagLocationHeight;
            angleToGoal = Math.toRadians(0 + tagLocationY);
            tagPose = fiducial.getRobotPoseTargetSpace();
            tagDistance = tagPose.getPosition().toUnit(DistanceUnit.INCH).z;
        }

    }
}