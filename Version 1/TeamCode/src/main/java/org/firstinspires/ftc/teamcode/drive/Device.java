package org.firstinspires.ftc.teamcode.drive;

public class Device {
    public String tag;
    public double lastSetPower;
    public double lastPower;
    public double kDeltaPower;
    public double kTime;
    public double kStatic;
    public long lastUpdateTime;
    public long lastTime;
    public double updateTimeMillis;
    public Device(double kTime, double kStatic, double kDeltaPower){
        this.tag = "motor";
        this.kTime = kTime;
        this.kStatic = kStatic;
        this.kDeltaPower = kDeltaPower;
        lastUpdateTime = System.nanoTime();
        lastSetPower = 0;
        lastPower = 0;
        updateTimeMillis = 1.584208401;
    }
    public Device(String sensorType, double kTime, double kStatic){
        this.tag = "sensor";
        this.kTime = kTime;
        this.kStatic = kStatic;
        lastUpdateTime = System.nanoTime();
        lastSetPower = 0;
        lastPower = 0;
        updateTimeMillis = 10;
        if (sensorType.equals("IMU")) {             updateTimeMillis = 7.0;}
        if (sensorType.equals("2MDistance")) {      updateTimeMillis = 15.38461538;}
        if (sensorType.equals("colorSensor1")) {    updateTimeMillis = 4.0;}
        if (sensorType.equals("colorSensor2")) {    updateTimeMillis = 7.692307692;}
        if (sensorType.equals("colorSensor3")) {    updateTimeMillis = 11.76470588;}
        if (sensorType.equals("touchSensor")) {     updateTimeMillis = 1.25;}
    }
    public void update(){
        lastUpdateTime = lastTime;
        lastSetPower = lastPower;
    }
    public double getImportance(){
        long currentTime = System.nanoTime();
        double timeFromLastUpdateMillis = (double)(currentTime - lastUpdateTime)/Math.pow(10,6);
        lastTime = currentTime;
        return timeFromLastUpdateMillis*kTime + kStatic;
    }
    public double getImportance(double power){
        long currentTime = System.nanoTime();
        double timeFromLastUpdateMillis = (double)(currentTime - lastUpdateTime)/Math.pow(10,6);
        lastTime = currentTime;
        double deltaPower = lastSetPower - power;
        lastPower = power;
        return timeFromLastUpdateMillis*kTime + deltaPower*kDeltaPower + kStatic;
    }
}
