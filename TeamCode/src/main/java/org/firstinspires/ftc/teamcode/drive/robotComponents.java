package org.firstinspires.ftc.teamcode.drive;

import java.util.ArrayList;

public class robotComponents {
    public ArrayList<Component> components;
    double wheelWidth;
    double wheelDiameter;
    double robotLength;
    double robotWidth;
    Point[] encoderPos;
    boolean drawnOdo = false;
    int componentNum = 0;
    int odoStart = 0;
    public robotComponents(){
        wheelWidth = 1.49606;
        wheelDiameter = 3.77953;
        robotLength = 15.5+1;
        robotWidth = 12.5+1;
        components = new ArrayList<Component>();
        Localizer l = new Localizer();
        encoderPos = new Point[l.encoders.length];
        for (int i = 0; i < encoderPos.length; i ++){
            encoderPos[i] = new Point(l.encoders[i].x,l.encoders[i].y);
        }
        robotBody();
        robotWheels();
        robotDirectionIndicator();
        //robotOdo();
    }
    public void robotOdo(){
        drawnOdo = true;
        odoStart = componentNum;
        for (int i = 0; i < encoderPos.length; i ++){
            Component odoPod = new Component(0.5);
            odoPod.color = "#f542f2";
            odoPod.p.add(new Point(encoderPos[i].x,encoderPos[i].y));
            components.add(odoPod); // 6 -> 6 + encoderPos.length-1
        }
    }
    public void setOdoColor(boolean isKnownPos){
        if (drawnOdo) {
            String color = "#f542f2";
            if (!isKnownPos) {
                color = "#ff0000";
            }
            for (int i = 0; i < encoderPos.length; i++) {
                components.get(i + odoStart).color = color;
            }
        }
    }
    public void robotDirectionIndicator() {
        Component direction = new Component();
        direction.color = "#707070";
        direction.p.add(new Point(0,0));
        direction.p.add(new Point(robotLength/2.0,0));
        components.add(direction); // 5
        componentNum ++;
    }
    public void robotBody() {
        Component robotBody = new Component();
        robotBody.color = "#707070";
        robotBody.p.add(new Point(robotLength/ 2.0,robotWidth/2.0));
        robotBody.p.add(new Point(robotLength/ 2.0,robotWidth/-2.0));
        robotBody.p.add(new Point(robotLength/-2.0,robotWidth/-2.0));
        robotBody.p.add(new Point(robotLength/-2.0,robotWidth/2.0));
        components.add(robotBody); // 0
        componentNum ++;
    }
    public void robotWheels(){
        for (int i = 0; i < 4; i ++) {
            Component wheel = new Component();
            wheel.color = "#707070";
            wheel.lineRadius=1;
            double n = 1.0;
            double b = 1.0;
            if (i%2 == 0) { n = -1.0; }
            if (i < 2) { b = -1.0; }
            double x = robotLength/2.0 - 0.5;
            double y = robotWidth/2.0 - 0.5;
            wheel.p.add(new Point((x)*b,(y)*n));
            wheel.p.add(new Point((x)*b,(y-wheelWidth)*n));
            wheel.p.add(new Point((x-wheelDiameter)*b,(y-wheelWidth)*n));
            wheel.p.add(new Point((x-wheelDiameter)*b,(y)*n));
            components.add(wheel); // 1, 2, 3, 4 respectively
            componentNum ++;
        }
    }
}
