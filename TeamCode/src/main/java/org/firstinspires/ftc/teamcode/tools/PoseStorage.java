package org.firstinspires.ftc.teamcode.tools;

import com.pedropathing.geometry.Pose;

public class PoseStorage {
    // Default = START_POSE del auto → TeleOp standalone funciona sin haber corrido auto.
    // En runtime, Comp.loop() sobreescribe esto cada ~5ms desde el primer ciclo.
    public static Pose currentPose = new Pose(123.0, 122.0, Math.toRadians(37));
}