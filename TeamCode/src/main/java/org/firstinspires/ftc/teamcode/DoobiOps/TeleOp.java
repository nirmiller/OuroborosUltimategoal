package org.firstinspires.ftc.teamcode.DoobiOps;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.DoobiLibraries.TeleLib;



@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleDrive")
public class TeleOp extends TeleLib {




    @Override
    public void loop() {
        arcadedrive();
        shooter();
        intake();
        magazine();
        wobbleGoal();
        killAll();
        telemetry.update();
    }

    @Override
    public void stop() {
        kill_count = 10;
        killAll();

        super.stop();

    }
}
