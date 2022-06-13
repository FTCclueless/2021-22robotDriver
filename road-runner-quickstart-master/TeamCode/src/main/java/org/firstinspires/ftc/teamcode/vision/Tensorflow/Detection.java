package org.firstinspires.ftc.teamcode.vision.Tensorflow;

import android.graphics.RectF;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

public class Detection {
    private Recognition RECOGNIZED_OBJECT;
    private float FOCAL_LENGTH;    //in mm
    private double SENSOR_HEIGHT;    //in mm

    public Detection(Recognition recognition, float focalLength, float verticalFOVAngle){
        RECOGNIZED_OBJECT = recognition;
        FOCAL_LENGTH = focalLength;
        SENSOR_HEIGHT = Math.tan(Math.toRadians(verticalFOVAngle / 2d)) * 2 * FOCAL_LENGTH;
    }

    public RectF getBoundingBox(){
        return new RectF(RECOGNIZED_OBJECT.getLeft(), RECOGNIZED_OBJECT.getTop(),
                RECOGNIZED_OBJECT.getRight(), RECOGNIZED_OBJECT.getBottom());
    }

    public String getLabel(){
        return RECOGNIZED_OBJECT.getLabel();
    }

    public float getBottom(){
        return RECOGNIZED_OBJECT.getBottom();
    }

    public float getLeft(){
        return RECOGNIZED_OBJECT.getLeft();
    }

    public float getRight(){
        return RECOGNIZED_OBJECT.getRight();
    }

    public float getTop(){
        return RECOGNIZED_OBJECT.getTop();
    }

    public float getWidth(){
        return RECOGNIZED_OBJECT.getWidth();
    }

    public float getHeight(){
        return RECOGNIZED_OBJECT.getHeight();
    }

    public float getImageWidth(){
        return RECOGNIZED_OBJECT.getImageWidth();
    }

    public float getImageHeight(){
        return RECOGNIZED_OBJECT.getImageHeight();
    }

    public float getConfidence(){
        return RECOGNIZED_OBJECT.getConfidence();
    }
}
