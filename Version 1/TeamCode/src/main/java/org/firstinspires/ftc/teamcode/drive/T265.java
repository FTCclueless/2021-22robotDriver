package org.firstinspires.ftc.teamcode.drive;

import android.content.Context;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.spartronics4915.lib.T265Camera;

public class T265 {
    T265Camera slmra;
    public T265(Pose2d p, double odometryCovariance, Context appContext){
        slmra = new T265Camera(new Transform2d(new Translation2d(p.getX()*0.0254,p.getY()*0.0254),new Rotation2d()),odometryCovariance,appContext);
    }
    public void start(){
        slmra.start();
    }
    public T265(Context appContext){
        this(new Pose2d(0,0),0.8,appContext);
    }
    public Pose2d getPoseEstimate(){
        T265Camera.CameraUpdate t265Data = slmra.getLastReceivedCameraUpdate();
        Pose2d poseInches = new Pose2d(t265Data.pose.getX()*-39.3701,t265Data.pose.getY()*-39.3701,t265Data.pose.getHeading());
        return poseInches;
    }
    public void sendOdometry(Pose2d relVel){
        slmra.sendOdometry(relVel.getX()*-0.0254,relVel.getY()*-0.0254);
    }
    public void end(){
        slmra.stop();
    }
}
