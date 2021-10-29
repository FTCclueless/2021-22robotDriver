package org.firstinspires.ftc.teamcode.trajectorysequence;

import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;
import com.acmerobotics.roadrunner.util.NanoClock;

import org.firstinspires.ftc.teamcode.drive.Component;
import org.firstinspires.ftc.teamcode.drive.Point;
import org.firstinspires.ftc.teamcode.drive.robotComponents;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.SequenceSegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TrajectorySegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TurnSegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.WaitSegment;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

@Config
public class TrajectorySequenceRunner {
    public static String COLOR_INACTIVE_TRAJECTORY = "#4caf507a";
    public static String COLOR_INACTIVE_TURN = "#7c4dff7a";
    public static String COLOR_INACTIVE_WAIT = "#dd2c007a";

    public static String COLOR_ACTIVE_TRAJECTORY = "#4CAF50";
    public static String COLOR_ACTIVE_TURN = "#7c4dff";
    public static String COLOR_ACTIVE_WAIT = "#dd2c00";

    public static int POSE_HISTORY_LIMIT = 100;

    private final TrajectoryFollower follower;

    private final PIDFController turnController;

    private final NanoClock clock;

    private TrajectorySequence currentTrajectorySequence;
    private double currentSegmentStartTime;
    private int currentSegmentIndex;
    private int lastSegmentIndex;

    private Pose2d lastPoseError = new Pose2d();

    Long lastLoopTime;

    private robotComponents t265Robot;
    private Pose2d t265Pose;
    private robotComponents threeWheelRobot;
    private Pose2d threeWheelPose;

    List<TrajectoryMarker> remainingMarkers = new ArrayList<>();

    private final FtcDashboard dashboard;
    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    public TrajectorySequenceRunner(TrajectoryFollower follower, PIDCoefficients headingPIDCoefficients) {
        this.follower = follower;

        t265Robot = null;
        t265Pose = null;
        threeWheelRobot = null;
        threeWheelPose = null;

        turnController = new PIDFController(headingPIDCoefficients);
        turnController.setInputBounds(0, 2 * Math.PI);

        clock = NanoClock.system();

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        lastLoopTime = System.nanoTime();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        currentTrajectorySequence = trajectorySequence;
        currentSegmentStartTime = clock.seconds();
        currentSegmentIndex = 0;
        lastSegmentIndex = -1;
    }

    public @Nullable DriveSignal update(Pose2d poseEstimate, Pose2d poseVelocity){
        return update(poseEstimate,poseVelocity, null);
    }

    public @Nullable
    DriveSignal update(Pose2d poseEstimate, Pose2d poseVelocity, robotComponents r) {
        long currentTime = System.nanoTime();
        double loopTime = (currentTime-lastLoopTime)/1000000000.0;
        lastLoopTime = currentTime;

        Pose2d targetPose = null;
        DriveSignal driveSignal = null;

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        SequenceSegment currentSegment = null;

        if (currentTrajectorySequence != null) {
            if (currentSegmentIndex >= currentTrajectorySequence.size()) {
                for (TrajectoryMarker marker : remainingMarkers) {
                    marker.getCallback().onMarkerReached();
                }

                remainingMarkers.clear();

                currentTrajectorySequence = null;
            }

            if (currentTrajectorySequence == null)
                return new DriveSignal();

            double now = clock.seconds();
            boolean isNewTransition = currentSegmentIndex != lastSegmentIndex;

            currentSegment = currentTrajectorySequence.get(currentSegmentIndex);

            if (isNewTransition) {
                currentSegmentStartTime = now;
                lastSegmentIndex = currentSegmentIndex;

                for (TrajectoryMarker marker : remainingMarkers) {
                    marker.getCallback().onMarkerReached();
                }

                remainingMarkers.clear();

                remainingMarkers.addAll(currentSegment.getMarkers());
                Collections.sort(remainingMarkers, (t1, t2) -> Double.compare(t1.getTime(), t2.getTime()));
            }

            double deltaTime = now - currentSegmentStartTime;

            if (currentSegment instanceof TrajectorySegment) {
                Trajectory currentTrajectory = ((TrajectorySegment) currentSegment).getTrajectory();

                if (isNewTransition)
                    follower.followTrajectory(currentTrajectory);

                if (!follower.isFollowing()) {
                    currentSegmentIndex++;

                    driveSignal = new DriveSignal();
                } else {
                    driveSignal = follower.update(poseEstimate, poseVelocity);
                    lastPoseError = follower.getLastError();
                }

                targetPose = currentTrajectory.get(deltaTime);
            } else if (currentSegment instanceof TurnSegment) {
                MotionState targetState = ((TurnSegment) currentSegment).getMotionProfile().get(deltaTime);

                turnController.setTargetPosition(targetState.getX());

                double correction = turnController.update(poseEstimate.getHeading());

                double targetOmega = targetState.getV();
                double targetAlpha = targetState.getA();

                lastPoseError = new Pose2d(0, 0, turnController.getLastError());

                Pose2d startPose = currentSegment.getStartPose();
                targetPose = startPose.copy(startPose.getX(), startPose.getY(), targetState.getX());

                driveSignal = new DriveSignal(
                        new Pose2d(0, 0, targetOmega + correction),
                        new Pose2d(0, 0, targetAlpha)
                );

                if (deltaTime >= currentSegment.getDuration()) {
                    currentSegmentIndex++;
                    driveSignal = new DriveSignal();
                }
            } else if (currentSegment instanceof WaitSegment) {
                lastPoseError = new Pose2d();

                targetPose = currentSegment.getStartPose();
                driveSignal = new DriveSignal();

                if (deltaTime >= currentSegment.getDuration()) {
                    currentSegmentIndex++;
                }
            }

            while (remainingMarkers.size() > 0 && deltaTime > remainingMarkers.get(0).getTime()) {
                remainingMarkers.get(0).getCallback().onMarkerReached();
                remainingMarkers.remove(0);
            }
        }

        poseHistory.add(poseEstimate);

        if (POSE_HISTORY_LIMIT > -1 && poseHistory.size() > POSE_HISTORY_LIMIT) {
            poseHistory.removeFirst();
        }
        packet.put("loopTime", loopTime);

        packet.put("xError", lastPoseError.getX());
        packet.put("yError", lastPoseError.getY());
        packet.put("headingError (deg)", Math.toDegrees(lastPoseError.getHeading()));
        packet.put("x", poseEstimate.getX());
        packet.put("y", poseEstimate.getY());
        packet.put("heading (deg)", Math.toDegrees(poseEstimate.getHeading()));
        packet.put("velX", poseVelocity.getX());
        packet.put("velY", poseVelocity.getY());
        packet.put("velHeading (deg)", Math.toDegrees(poseVelocity.getHeading()));

        draw(fieldOverlay, currentTrajectorySequence, currentSegment, targetPose, poseEstimate, r);

        dashboard.sendTelemetryPacket(packet);

        return driveSignal;
    }

    private void draw(
            Canvas fieldOverlay,
            TrajectorySequence sequence, SequenceSegment currentSegment,
            Pose2d targetPose, Pose2d poseEstimate,
            robotComponents r
    ) {
        if (sequence != null) {
            for (int i = 0; i < sequence.size(); i++) {
                SequenceSegment segment = sequence.get(i);

                if (segment instanceof TrajectorySegment) {
                    fieldOverlay.setStrokeWidth(1);
                    fieldOverlay.setStroke(COLOR_INACTIVE_TRAJECTORY);

                    DashboardUtil.drawSampledPath(fieldOverlay, ((TrajectorySegment) segment).getTrajectory().getPath());
                } else if (segment instanceof TurnSegment) {
                    Pose2d pose = segment.getStartPose();

                    fieldOverlay.setFill(COLOR_INACTIVE_TURN);
                    fieldOverlay.fillCircle(pose.getX(), pose.getY(), 2);
                } else if (segment instanceof WaitSegment) {
                    Pose2d pose = segment.getStartPose();

                    fieldOverlay.setStrokeWidth(1);
                    fieldOverlay.setStroke(COLOR_INACTIVE_WAIT);
                    fieldOverlay.strokeCircle(pose.getX(), pose.getY(), 3);
                }
            }
        }

        if (currentSegment != null) {
            if (currentSegment instanceof TrajectorySegment) {
                Trajectory currentTrajectory = ((TrajectorySegment) currentSegment).getTrajectory();

                fieldOverlay.setStrokeWidth(1);
                fieldOverlay.setStroke(COLOR_ACTIVE_TRAJECTORY);

                DashboardUtil.drawSampledPath(fieldOverlay, currentTrajectory.getPath());
            } else if (currentSegment instanceof TurnSegment) {
                Pose2d pose = currentSegment.getStartPose();

                fieldOverlay.setFill(COLOR_ACTIVE_TURN);
                fieldOverlay.fillCircle(pose.getX(), pose.getY(), 3);
            } else if (currentSegment instanceof WaitSegment) {
                Pose2d pose = currentSegment.getStartPose();

                fieldOverlay.setStrokeWidth(1);
                fieldOverlay.setStroke(COLOR_ACTIVE_WAIT);
                fieldOverlay.strokeCircle(pose.getX(), pose.getY(), 3);
            }
        }

        if (targetPose != null) {
            fieldOverlay.setStrokeWidth(1);
            fieldOverlay.setStroke("#4CAF50");
            DashboardUtil.drawRobot(fieldOverlay, targetPose);
        }

        fieldOverlay.setStroke("#3F51B5");
        DashboardUtil.drawPoseHistory(fieldOverlay, poseHistory);

        if (r != null) {
            drawRobot(fieldOverlay,r,poseEstimate);
        }
        else {
            fieldOverlay.setStroke("#3F51B5");
            DashboardUtil.drawRobot(fieldOverlay, poseEstimate);
        }

        if (t265Pose != null) {
            drawRobot(fieldOverlay, t265Robot, t265Pose);
        }

        if (threeWheelPose != null) {
            drawRobot(fieldOverlay, threeWheelRobot, threeWheelPose);
        }
    }
    public void updateT265(Pose2d p){
        t265Pose = p;
    }
    public void updateThreeWheelPose(Pose2d p){
        threeWheelPose = p;
    }
    public void initT265Robot(){
        t265Pose = new Pose2d(0,0,0);
        t265Robot = new robotComponents(false,"#00e5ff");
    }
    public void initThreeWheelRobot(){
        threeWheelPose = new Pose2d(0,0,0);
        threeWheelRobot = new robotComponents(false,"#55ff66");
    }
    public void drawRobot(Canvas fieldOverlay, robotComponents r, Pose2d poseEstimate){
        for (Component c : r.components){
            fieldOverlay.setStrokeWidth(c.lineRadius);
            fieldOverlay.setStroke(c.color);
            if (c.p.size() == 1){
                drawPoint(fieldOverlay,c.p.get(0),c.radius,poseEstimate);
            }
            else {
                for (int i = 1; i < c.p.size() + 1; i++) {
                    drawPoint(fieldOverlay, c.p.get(i % c.p.size()), c.radius, poseEstimate);
                    drawLine(fieldOverlay, c.p.get(i % c.p.size()), c.p.get((i - 1)%c.p.size()), poseEstimate);
                }
            }
        }
    }
    public void drawLine(Canvas c, Point p, Point p2, Pose2d pose){
        double x1 = Math.cos(pose.getHeading())*(p.x+Math.signum(p.x-p2.x)*0.5) - Math.sin(pose.getHeading())*(p.y+Math.signum(p.y-p2.y)*0.5) + pose.getX();
        double y1 = Math.cos(pose.getHeading())*(p.y+Math.signum(p.y-p2.y)*0.5) + Math.sin(pose.getHeading())*(p.x+Math.signum(p.x-p2.x)*0.5) + pose.getY();
        double x2 = Math.cos(pose.getHeading())*(p2.x+Math.signum(p2.x-p.x)*0.5) - Math.sin(pose.getHeading())*(p2.y+Math.signum(p2.y-p.y)*0.5) + pose.getX();
        double y2 = Math.cos(pose.getHeading())*(p2.y+Math.signum(p2.y-p.y)*0.5) + Math.sin(pose.getHeading())*(p2.x+Math.signum(p2.x-p.x)*0.5) + pose.getY();
        c.strokeLine(x1,y1,x2,y2);
    }
    public void drawPoint(Canvas c, Point p, double radius, Pose2d pose){
        if (radius != 0){
            double x = Math.cos(pose.getHeading())*p.x - Math.sin(pose.getHeading())*p.y + pose.getX();
            double y = Math.cos(pose.getHeading())*p.y + Math.sin(pose.getHeading())*p.x + pose.getY();
            c.strokeCircle(x,y,radius);
        }
    }

    public Pose2d getLastPoseError() {
        return lastPoseError;
    }

    public boolean isBusy() {
        return currentTrajectorySequence != null;
    }
}
