package org.firstinspires.ftc.teamcode.drive;

import java.util.ArrayList;

public class robotComponents {
    public ArrayList<Component> components;
    double wheelWidth;
    double wheelDiameter;
    double robotLength;
    double robotWidth;
    Point[] encoderPos;
    public robotComponents(){
        wheelWidth = 1.5;
        wheelDiameter = 4;
        robotLength = 16;
        robotWidth = 12;
        components = new ArrayList<Component>();
        Localizer l = new Localizer();
        encoderPos = new Point[l.encoders.length];
        for (int i = 0; i < encoderPos.length; i ++){
            encoderPos[i] = new Point(l.encoders[i].x,l.encoders[i].y);
        }
        robotBody();
        robotWheels();
        robotDirectionIndicator();
        robotOdo();
    }
    public void robotOdo(){
        for (int i = 0; i < encoderPos.length; i ++){
            Component odoPod = new Component();
            odoPod.color = "#f542f2";
            odoPod.p.add(new Point((encoderPos[i].x - 1),(encoderPos[i].y + 1)));
            odoPod.p.add(new Point((encoderPos[i].x - 1),(encoderPos[i].y - 1)));
            odoPod.p.add(new Point((encoderPos[i].x + 1),(encoderPos[i].y - 1)));
            odoPod.p.add(new Point((encoderPos[i].x + 1),(encoderPos[i].y - 1)));
            components.add(odoPod); // 7 -> 7 + encoderPos.length-1
        }
    }
    public void setOdoColor(boolean isKnownPos){
        String color = "#f542f2";
        if (!isKnownPos){
            color = "#f542f2";
        }
        for (int i = 0; i < encoderPos.length; i ++) {
            components.get(i+7).color = color;
        }
    }
    public void robotDirectionIndicator() {
        Component direction = new Component();
        direction.color = "#fcba03";
        direction.p.add(new Point(0,0));
        direction.p.add(new Point(0,2.5));
        components.add(direction); // 5
        Component circle = new Component(3);
        circle.color = "#fcba03";
        circle.p.add(new Point(0,0));
        components.add(circle); // 6
    }
    public void robotBody() {
        Component robotBody = new Component();
        robotBody.color = "#03bafc";
        robotBody.p.add(new Point(robotLength/ 2.0,robotWidth/2.0));
        robotBody.p.add(new Point(robotLength/ 2.0,robotWidth/-2.0));
        robotBody.p.add(new Point(robotLength/-2.0,robotWidth/-2.0));
        robotBody.p.add(new Point(robotLength/-2.0,robotWidth/2.0));
        components.add(robotBody); // 0
    }
    public void robotWheels(){
        for (int i = 0; i < 4; i ++) {
            Component wheel = new Component();
            wheel.color = "#03fcd7";
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
        }
    }
}
