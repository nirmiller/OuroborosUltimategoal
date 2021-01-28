package org.firstinspires.ftc.teamcode.DoobiOps;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.DoobiLibraries.BackupVision;
import org.firstinspires.ftc.teamcode.DoobiLibraries.RegressBerryPie;

@Autonomous(group = "Auto", name = "VisionTesting")
public class VisionTrollAuto extends LinearOpMode {

    RegressBerryPie vision;
    @Override
    public void runOpMode() throws InterruptedException {

        vision = new RegressBerryPie(this);
        waitForStart();
        telemetry.addData("position", vision.pie());

    }
}
