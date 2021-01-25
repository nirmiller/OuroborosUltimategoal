package org.firstinspires.ftc.teamcode.DoobiOps;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.DoobiLibraries.BackupVision;

@Autonomous(group = "Auto", name = "VisionTesting")
public class VisionTrollAuto extends LinearOpMode {

    BackupVision vision;
    @Override
    public void runOpMode() throws InterruptedException {

        vision = new BackupVision(this);

        waitForStart();
        //TODO: Test and note the values of the pixels
        vision.senseBlue(this);

    }
}
