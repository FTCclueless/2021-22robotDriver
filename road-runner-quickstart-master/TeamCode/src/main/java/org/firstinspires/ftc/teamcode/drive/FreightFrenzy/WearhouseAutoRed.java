package org.firstinspires.ftc.teamcode.drive.FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(group = "Auto")
public class WearhouseAutoRed extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        WearhouseAutoMaster a = new WearhouseAutoMaster(false,true, new SampleMecanumDrive(hardwareMap));
        //TODO: Implement ML
        a.capNum = 2;
    }
}