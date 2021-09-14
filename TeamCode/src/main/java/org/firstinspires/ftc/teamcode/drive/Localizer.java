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
        offsetHeading = 0.0;
        encoders = new Encoder[4];
        encoders[0] = new Encoder(new Vector2d(0.125,-7.18),1.0); // the difference between the two encoders is 13.7614173 in the old value was 13.875
        encoders[1] = new Encoder(new Vector2d(0.125,6.58),-1.0); // the width of the encoder is 0.6023622 in that is why the right encoder is that much further from center
        encoders[2] = new Encoder(new Vector2d(-6,-1.25),-1.0);
        encoders[3] = new Encoder(new Vector2d(1.5,-1.25),1.0);
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
        odoHeading = (encoders[0].getCurrentDist() - encoders[1].getCurrentDist())/(Math.abs(encoders[1].y-encoders[0].y));
        double sideHeading = (encoders[3].getCurrentDist() - encoders[2].getCurrentDist())/(Math.abs(encoders[3].y-encoders[2].y));
        double heading = odoHeading + offsetHeading;
        Log.e("Heading Difference", (odoHeading-sideHeading) + "");

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
        double deltaFrontHeading = (deltaRight - deltaLeft)/(encoders[1].y-encoders[0].y); // this works because S = theta*r. The y is the r and is negative for the left, therefore no need for a minus sign
        double relDeltaX = 0;
        double relDeltaY = 0;
        double relVelHeading = deltaFrontHeading/loopTime;

        if (deltaRight - deltaLeft == 0){
            relDeltaX += (deltaLeft + deltaRight)/2.0;
        }
        else{
            double forwardR = (deltaLeft*encoders[1].y - deltaRight*encoders[0].y)/(deltaRight - deltaLeft); // calculating the turn radius for the forward encoders
            relDeltaX += Math.sin(deltaFrontHeading)*forwardR;
            relDeltaY += (1.0 - Math.cos(deltaFrontHeading))*forwardR;
        }
        //double deltaSideHeading = (deltaBack - deltaFront)/(encoders[3].x -encoders[2].x);
        if (deltaBack - deltaFront == 0){
            relDeltaY += (deltaFront + deltaBack)/2.0;
        }
        else{
            double sideR = (deltaBack*encoders[2].x - deltaFront*encoders[3].x)/(deltaFront - deltaBack); // calculating the turn radius for the side encoders
            relDeltaX += (1.0 - Math.cos(deltaFrontHeading))*sideR;
            relDeltaY += Math.sin(deltaFrontHeading)*sideR;
        }
        double eHeading = heading;// - deltaFrontHeading/2.0; // This finds the heading midway between last heading and this heading
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
