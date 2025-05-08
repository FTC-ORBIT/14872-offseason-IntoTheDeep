package org.firstinspires.ftc.teamcode.OrbitHardware.OrbitCams.OrbitWebcam.TempVision.aprilTags;

import org.firstinspires.ftc.teamcode.OrbitUtils.Pose2D;
import org.opencv.core.Point;

import java.util.ArrayList;

public class TagInfo {
        public final String name;
        public final int id;
        public final float size;
        public final ArrayList<Point> points;
        public final Pose2D deltaPose;
        public final Pose2D tagPose;


        public TagInfo(final String name,final int id, final float size, final ArrayList<Point> points,final Pose2D deltaPose,final Pose2D tagPose) {
            this.name = name;
            this.id = id;
            this.size = size;
            this.points = points;
            this.deltaPose = deltaPose;
            this.tagPose = tagPose;
        }

}
