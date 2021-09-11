package org.firstinspires.ftc.teamcode.drive;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

public class Localizer implements com.acmerobotics.roadrunner.localization.Localizer {
    Encoder[] encoders;
    double odoHeading;
    double offsetHeading;
    Pose2d currentPose = new Pose2d(0,0,0);
    Pose2d currentVel = new Pose2d(0,0,0);
    double x = 0;
    double y = 0;
    long lastTime = System.nanoTime();
    double startHeadingOffset = 0;

    public Localizer(){
        encoders = new Encoder[4];
        encoders[0] = new Encoder(new Vector2d(),1.0);
        encoders[1] = new Encoder(new Vector2d(),1.0);
        encoders[2] = new Encoder(new Vector2d(),1.0);
        encoders[3] = new Encoder(new Vector2d(),1.0);
    }

    public void updateEncoders(int[] encoders){
        for (int i = 0; i < this.encoders.length; i ++){
            this.encoders[i].update(encoders[i]);
        }
    }

    @NotNull
    @Override


    public Pose2d getPoseEstimate() {
        return currentPose;
    }

    public void updateHeading(double imuHeading){
        double headingDif = imuHeading-odoHeading;
        while (headingDif > Math.toRadians(180)){
            headingDif -= Math.toRadians(360);
        }
        while (headingDif < Math.toRadians(-180)){
            headingDif += Math.toRadians(360);
        }
        double w = 0.1;
        offsetHeading = (offsetHeading)*w + headingDif*(1.0-w);

    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
        x = pose2d.getX();
        y = pose2d.getY();
        startHeadingOffset = pose2d.getHeading() - currentPose.getHeading();
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return currentVel;
    }

    @Override
    public void update() {
        double odoHeading = encoders[0].getCurrentDist()/encoders[0].y + encoders[1].getCurrentDist()/encoders[1].y;
        double sideHeading = encoders[2].getCurrentDist()/encoders[2].y + encoders[3].getCurrentDist()/encoders[3].y;
        Log.e("odoHeading",odoHeading + "");
        Log.e("sideHeading",sideHeading + "");
        double heading = odoHeading + offsetHeading;

        long currentTime = System.nanoTime();
        double loopTime = (currentTime-lastTime)/1000000000.0;
        lastTime = currentTime;

        /*
        ___________________________                           x+
        |                         |                           ^
        |                         |                           |
        |                         |                           |
        | ^        <[E4]        ^ |                           |
        |[E2]                 [E1]|                           |
        |                         |                           |
        |                         |                           |
        |          <[E3]          |                           |
        |_________________________|+y<------------------------|
         */

        double deltaRight = encoders[0].getDelta();     //E1
        double deltaLeft = encoders[1].getDelta();      //E2
        double deltaBack = encoders[2].getDelta();      //E3
        double deltaFront = encoders[3].getDelta();     //E4
        double deltaHeading = deltaRight/encoders[0].y + deltaLeft/encoders[1].y; // this works because S = theta*r. The y is the r and is negative for the left, therefore no need for a minus sign

        double relDeltaX;
        double relDeltaY;
        double relVelHeading = deltaHeading/loopTime;
        if (deltaHeading == 0){
            relDeltaY = (deltaFront + deltaBack)/2;
            relDeltaX = (deltaLeft + deltaRight)/2;
        }
        else{
            double forwardR = (deltaRight*encoders[0].y - deltaLeft*encoders[1].y)/(deltaRight - deltaLeft); // calculating the turn radius for the forward encoders
            double sideR = (deltaBack*encoders[2].x - deltaFront*encoders[3].x)/(deltaBack - deltaFront); // calculating the turn radius for the side encoders
            relDeltaX = Math.sin(deltaHeading)*forwardR + (1.0 - Math.cos(deltaHeading))*sideR;
            relDeltaY = Math.sin(deltaHeading)*sideR + (1.0 - Math.cos(deltaHeading))*forwardR;
        }


        double eHeading = odoHeading - deltaHeading/2.0; // This finds the heading midway between last heading and this heading
        x += relDeltaX*Math.cos(eHeading) - relDeltaY*Math.sin(eHeading);
        y += relDeltaY*Math.cos(eHeading) + relDeltaX*Math.sin(eHeading);

        double w = 0.35; // This is a Kalman Filter to help make sure that the velocities stay relatively stable
        double newVelX = ((relDeltaX)/loopTime - currentVel.getX())*w + currentVel.getX()*(1.0-w);//X-lastX
        double newVelY = ((relDeltaY)/loopTime - currentVel.getY())*w + currentVel.getY()*(1.0-w);//y-lastY
        if (Math.abs(newVelX) < 1){
            newVelX =0;
        }
        if (Math.abs(newVelY) < 1){
            newVelY =0;
        }
        currentVel = new Pose2d(newVelX,newVelY,relVelHeading);
        currentPose = new Pose2d(x,y,heading);
    }
}
