package org.firstinspires.ftc.teamcode.drive.FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(group = "Auto")
public class WearhouseAutoBlue extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        WearhouseAutoMaster a = new WearhouseAutoMaster(true);
    }
}