package org.firstinspires.ftc.teamcode.DoobiOps;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.DoobiLibraries.BackupVision;
import org.firstinspires.ftc.teamcode.DoobiLibraries.RegressBerryPie;

@Autonomous(group = "Auto", name = "VisionTesting")
public class VisionTrollAuto extends LinearOpMode {

    RegressBerryPie vision;
    BackupVision backupVision;
    @Override
    public void runOpMode() throws InterruptedException {
        //TODO: test and see if this works. If not, try running backup vision (you may have to adjust some values)
        //vision = new RegressBerryPie(this);
        backupVision = new BackupVision(this);
        waitForStart();
        //.telemetry.addData("position", vision.pie());
        telemetry.addData("position", backupVision.senseBlue(this));
        telemetry.update();

    }
}

