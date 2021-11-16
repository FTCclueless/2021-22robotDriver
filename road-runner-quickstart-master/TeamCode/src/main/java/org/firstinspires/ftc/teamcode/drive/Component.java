package org.firstinspires.ftc.teamcode.drive;
import java.util.ArrayList;

public class Component {
    public ArrayList<Point> p;
    public double radius;
    public int lineRadius;
    public String color;
    public Component(){
        p = new ArrayList<Point>();
        lineRadius = 1;
        radius = 0;
        color = "#000000";
    }
    public Component(double Radius){
        radius = Radius;
        lineRadius = 1;
        p = new ArrayList<Point>();
        color = "#000000";
    }
}