package org.firstinspires.ftc.teamcode.drive;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.ArrayList;

public class Localizer implements com.acmerobotics.roadrunner.localization.Localizer {
    public Encoder[] encoders;
    double odoHeading;
    double offsetHeading;
    public boolean updatPose;
    double gain = 1.0;
    Pose2d currentPose = new Pose2d(0,0,0);
    Pose2d currentVel = new Pose2d(0,0,0);
    Pose2d relCurrentVel = new Pose2d(0,0,0);
    double x = 0;
    double y = 0;
    long lastTime = System.nanoTime();
    double startHeadingOffset = 0;
    ArrayList<Pose2d> poseHistory = new ArrayList<Pose2d>();
    ArrayList<Pose2d> relHistory = new ArrayList<Pose2d>();
    ArrayList<Double> loopTimes = new ArrayList<Double>();
    double velLoop = 10;

    public Localizer(){
        for (int i = 0; i < velLoop; i ++){
            poseHistory.add(new Pose2d(0,0,0));
            relHistory.add(new Pose2d(0,0,0));
            loopTimes.add(0.001);
        }
        updatPose = true;
        offsetHeading = 0.0;
        encoders = new Encoder[4];
        encoders[0] = new Encoder(new Vector2d(0.125,-4.798502587509156),1.0);
        encoders[1] = new Encoder(new Vector2d(0.125,4.746510815258789),  -1.0);
        encoders[2] = new Encoder(new Vector2d(2.311807240850830,-3), -1.0);
        encoders[3] = new Encoder(new Vector2d(6.531451276593018,-3),  1.0);
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
    @Nullable
    public Pose2d getRelPoseVelocity(){return relCurrentVel;}

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

        double deltaHeading = (deltaRight - deltaLeft)/Math.abs(encoders[1].y-encoders[0].y);
        // this works because S = theta*r. The y is the r and is negative for the left, therefore no need for a minus sign
        double relDeltaY = deltaBack - deltaHeading*encoders[2].x;
        if (encoders.length == 4){
            double deltaFront = encoders[3].getDelta();     //E4
            relDeltaY = (deltaBack + deltaFront)/2 - deltaHeading*Math.abs(encoders[3].x-encoders[2].x);
        }
        odoHeading = (encoders[0].getCurrentDist() - encoders[1].getCurrentDist())/(Math.abs(encoders[1].y-encoders[0].y));
        double heading = odoHeading + offsetHeading + startHeadingOffset;
        double relDeltaX = (deltaLeft*encoders[0].y - deltaRight*encoders[1].y)/(encoders[0].y-encoders[1].y);
        if (updatPose) {
            if (deltaHeading != 0) { // this avoids the issue where deltaHeading = 0 and then it goes to undefined. This effectively does L'Hopital's
                double r1 = relDeltaX / deltaHeading;
                double r2 = relDeltaY / deltaHeading;
                relDeltaX = Math.sin(deltaHeading) * r1 + (1.0-Math.cos(deltaHeading)) * r2;
                relDeltaY = (1.0 - Math.cos(deltaHeading)) * r1 + Math.sin(deltaHeading) * r2;
            }
            double lastHeading = heading-deltaHeading;
            x += relDeltaX * Math.cos(lastHeading) - relDeltaY * Math.sin(lastHeading);
            y += relDeltaY * Math.cos(lastHeading) + relDeltaX * Math.sin(lastHeading);
        }
        else{
            relDeltaX = 0;
            relDeltaY = 0;
            deltaHeading = 0;
        }
        relHistory.add(0,new Pose2d(relDeltaX,relDeltaY,deltaHeading));
        poseHistory.add(0,new Pose2d(x,y,heading));
        loopTimes.add(0,loopTime);
        double totalTime = 0;
        Pose2d total = new Pose2d(0,0,0);
        int n = loopTimes.size()-1;
        for (int i = 0; i < n+1; i ++){
            totalTime += loopTimes.get(i);
            total = new Pose2d(total.getX()+relHistory.get(i).getX(),total.getY()+relHistory.get(i).getY(),total.getHeading()+relHistory.get(i).getHeading());
        }
        Pose2d delta = new Pose2d(
                poseHistory.get(0).getX()-poseHistory.get(n).getX(),
                poseHistory.get(0).getY()-poseHistory.get(n).getY(),
                poseHistory.get(0).getHeading()-poseHistory.get(n).getHeading()
        );
        Pose2d currentVelP    = new Pose2d(delta.getX()/totalTime,delta.getY()/totalTime, delta.getHeading()/totalTime);
        Pose2d relCurrentVelP = new Pose2d(total.getX()/totalTime,total.getY()/totalTime, total.getHeading()/totalTime);

        currentVel = new Pose2d(
                currentVelP.getX()*gain + currentVel.getX()*(1.0-gain),
                currentVelP.getY()*gain + currentVel.getY()*(1.0-gain),
                currentVelP.getHeading()*gain + currentVel.getHeading()*(1.0-gain)
        );
        relCurrentVel = new Pose2d(
                relCurrentVelP.getX()*gain + relCurrentVel.getX()*(1.0-gain),
                relCurrentVelP.getY()*gain + relCurrentVel.getY()*(1.0-gain),
                relCurrentVelP.getHeading()*gain + relCurrentVel.getHeading()*(1.0-gain)
        );
        currentPose = new Pose2d(x,y,heading);

        relHistory.remove(n);
        poseHistory.remove(n);
        loopTimes.remove(n);
    }
}
