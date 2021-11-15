package org.firstinspires.ftc.teamcode.drive;

import android.content.Context;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.spartronics4915.lib.T265Camera;

public class T265 {
    public T265Camera slmra = null;
    static Transform2d transform2d = new Transform2d();
    public void T265Init(Pose2d p, double odometryCovariance, Context appContext){
        if (slmra == null) {
            transform2d = new Transform2d(new Translation2d(p.getX() * 0.0254, p.getY() * 0.0254), new Rotation2d());
            slmra = new T265Camera(new Transform2d(new Translation2d(), new Rotation2d()), odometryCovariance, appContext);
        }
    }
    public void updateCovariance(double odometryCovariance, Context appContext){
        slmra = new T265Camera(transform2d,odometryCovariance,appContext);
    }
    public void start(){
        if (!slmra.isStarted()) {
            slmra.start();
        }
    }
    public T265Camera.CameraUpdate getT265Data(){
        return slmra.getLastReceivedCameraUpdate();
    }
    public String getT265Confidence(){
        T265Camera.CameraUpdate t265Data = slmra.getLastReceivedCameraUpdate();
        return t265Data.confidence.name();
    }
    public Pose2d getPoseEstimate(){
        T265Camera.CameraUpdate t265Data = slmra.getLastReceivedCameraUpdate();
        double x = t265Data.pose.getX() * -1
                + transform2d.getTranslation().getX() * (1 - Math.cos(t265Data.pose.getHeading()))
                + transform2d.getTranslation().getY() * Math.sin(t265Data.pose.getHeading());
        double y = t265Data.pose.getY() * -1
                + transform2d.getTranslation().getY() * (1 - Math.cos(t265Data.pose.getHeading()))
                - transform2d.getTranslation().getX() * Math.sin(t265Data.pose.getHeading());
        return new Pose2d(x * 39.3701,y * 39.3701,t265Data.pose.getHeading());
    }
    public Pose2d getRelVelocity(){
        T265Camera.CameraUpdate t265Data = slmra.getLastReceivedCameraUpdate();
        ChassisSpeeds a = t265Data.velocity;
        return new Pose2d(a.vxMetersPerSecond*-39.3701,a.vyMetersPerSecond*-39.3701,a.omegaRadiansPerSecond);
    }
    public void sendOdometry(Pose2d relVel){
        double relX = relVel.getX()*-0.0254 - relVel.getHeading()*transform2d.getTranslation().getY();
        if (Math.abs(relX) <= 0.01){
            relX = 0;
        }
        double relY = relVel.getY()*-0.0254 + relVel.getHeading()*transform2d.getTranslation().getX();
        if (Math.abs(relY) <= 0.01){
            relY = 0;
        }
        slmra.sendOdometry(relX,relY);
    }
    public void end(){
        slmra.stop();
    }
}
