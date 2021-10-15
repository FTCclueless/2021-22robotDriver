package org.firstinspires.ftc.teamcode.drive;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

public class Localizer implements com.acmerobotics.roadrunner.localization.Localizer {
    public Encoder[] encoders;
    double odoHeading;
    double offsetHeading;
    public boolean updatPose;
    Pose2d currentPose = new Pose2d(0,0,0);
    Pose2d currentVel = new Pose2d(0,0,0);
    double x = 0;
    double y = 0;
    long lastTime = System.nanoTime();
    double startHeadingOffset = 0;

    public Localizer(){
        updatPose = true;
        offsetHeading = 0.0;
        encoders = new Encoder[3];
        encoders[0] = new Encoder(new Vector2d(0.125,-4.9781),1.0);
        encoders[1] = new Encoder(new Vector2d(0.125,4.55088),  -1.0);
        encoders[2] = new Encoder(new Vector2d(-6.50377,-1.25), -1.0);
        //encoders[3] = new Encoder(new Vector2d(1.51006,-1.25),  -1.0);
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
        double headingDif = imuHeading-(odoHeading+offsetHeading);
        while (headingDif > Math.toRadians(180)){
            headingDif -= Math.toRadians(360);
        }
        while (headingDif < Math.toRadians(-180)){
            headingDif += Math.toRadians(360);
        }
        double w = 1.0;
        offsetHeading += headingDif*w;

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
        long currentTime = System.nanoTime();
        double loopTime = (currentTime-lastTime)/1000000000.0;
        lastTime = currentTime;

        /*
        ___________________________                           x+
        |                         |                           ^
        |                         |                           |
        |                         |                           |
        | ^          <[E4]      ^ |                           |
        |[E2]      (0,0)      [E1]|                           |
        |                         |                           |
        |                         |                           |
        |            <[E3]        |                           |
        |_________________________|+y<------------------------|
         */

        double deltaRight = encoders[0].getDelta();     //E1
        double deltaLeft = encoders[1].getDelta();      //E2
        double deltaBack = encoders[2].getDelta();      //E3

        double deltaFrontHeading = (deltaRight - deltaLeft)/Math.abs(encoders[1].y-encoders[0].y); // this works because S = theta*r. The y is the r and is negative for the left, therefore no need for a minus sign

        double deltaHeading = deltaFrontHeading;
        double relDeltaY = deltaBack - deltaHeading*encoders[2].x;
        if (encoders.length == 4){
            double deltaFront = encoders[3].getDelta();     //E4
            relDeltaY = (deltaBack*encoders[3].x - deltaFront*encoders[2].x)/(encoders[3].x-encoders[2].x);

            //double deltaSideHeading = (deltaBack - deltaFront)/Math.abs(encoders[3].x-encoders[2].x);
            //double w1 = 0.9;
            //deltaHeading = deltaFrontHeading*(w1) + deltaSideHeading*(1.0-w1);
            //relDeltaY = (deltaFront + deltaBack)/2.0 - deltaHeading*(encoders[2].x+encoders[3].x)/2.0;
        }
        odoHeading = (encoders[0].getCurrentDist() - encoders[1].getCurrentDist())/(Math.abs(encoders[1].y-encoders[0].y));
        double heading = odoHeading + offsetHeading + startHeadingOffset;


        //double relDeltaX = (deltaLeft + deltaRight)/2.0 + deltaHeading*(encoders[1].y+encoders[0].y)/2.0;
        double relDeltaX = (deltaLeft*encoders[0].y - deltaRight*encoders[1].y)/(encoders[0].y-encoders[1].y);

        double simLoops = 100.0;
        double simHeading = heading - deltaHeading;
        double simDeltaX = relDeltaX/simLoops;
        double simDeltaY = relDeltaY/simLoops;
        for (int i = 0; i < simLoops; i ++){
            simHeading += deltaHeading/(2.0*simLoops);
            if (updatPose) {
                x += simDeltaX * Math.cos(simHeading) - simDeltaY * Math.sin(simHeading);
                y += simDeltaY * Math.cos(simHeading) + simDeltaX * Math.sin(simHeading);
            }
            simHeading += deltaHeading/(2.0*simLoops);
        }

        double w2 = 0.05; // This is a Kalman Filter to help make sure that the velocities stay relatively stable
        double newVelX = ((relDeltaX)/loopTime)*w2 + currentVel.getX()*(1.0-w2);//X-lastX
        double newVelY = ((relDeltaY)/loopTime)*w2 + currentVel.getY()*(1.0-w2);//y-lastY
        double relVelHeading = deltaHeading/loopTime;
        double newRelVelHeading = (relVelHeading)*w2 + currentVel.getHeading()*(1.0-w2);
        if (Math.abs(newVelX) < 0.1){
            newVelX =0;
        }
        if (Math.abs(newVelY) < 0.1){
            newVelY =0;
        }
        if (Math.abs(newRelVelHeading) < 0.2/Math.abs(encoders[1].y-encoders[0].y)){
            newRelVelHeading = 0;
        }
        currentVel = new Pose2d(newVelX,newVelY,newRelVelHeading);
        currentPose = new Pose2d(x,y,heading);
    }
}
