package org.firstinspires.ftc.teamcode.drive;

import android.content.Context;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.spartronics4915.lib.T265Camera;

public class T265 {
    public static T265Camera slmra;
    static boolean startUp = false;
    static Transform2d transform2d = new Transform2d();
    public T265(Pose2d p, double odometryCovariance, Context appContext){
        startUp = true;
        transform2d = new Transform2d(new Translation2d(p.getX() * 0.0254, p.getY() * 0.0254), new Rotation2d());
        slmra = new T265Camera(new Transform2d(new Translation2d(p.getX()*0.0254,p.getY()*0.0254),new Rotation2d()),odometryCovariance,appContext);
    }
    public static void T265Init(Pose2d p, double odometryCovariance, Context appContext){
        transform2d = new Transform2d(new Translation2d(p.getX() * 0.0254, p.getY() * 0.0254), new Rotation2d());
        if (!startUp) {
            slmra = new T265Camera(new Transform2d(new Translation2d(p.getX() * 0.0254, p.getY() * 0.0254), new Rotation2d()), odometryCovariance, appContext);
        }
        startUp = true;
    }
    public static void updateCovariance(double odometryCovariance, Context appContext){
        slmra = new T265Camera(transform2d,odometryCovariance,appContext);
    }
    public static void start(){
        slmra.start();
    }
    public static Pose2d getPoseEstimate(){
        T265Camera.CameraUpdate t265Data = slmra.getLastReceivedCameraUpdate();
        Pose2d poseInches = new Pose2d(t265Data.pose.getX()*-39.3701,t265Data.pose.getY()*-39.3701,t265Data.pose.getHeading());
        return poseInches;
    }
    public static void sendOdometry(Pose2d relVel){
        slmra.sendOdometry(relVel.getX()*-0.0254,relVel.getY()*-0.0254);
    }
    public static void end(){
        slmra.stop();
    }
}
