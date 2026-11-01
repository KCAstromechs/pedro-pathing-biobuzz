package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Pose;

public class PoseMirror {
    public static Pose mirror(Pose pose) {
        return new Pose(141.5 - pose.x(), pose.y(), Math.PI - pose.heading());
    }
}