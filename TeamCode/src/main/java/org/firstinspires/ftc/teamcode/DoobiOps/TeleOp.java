package org.firstinspires.ftc.teamcode.DoobiOps;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.DoobiLibraries.TeleLib;



@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleDrive")
public class TeleOp extends TeleLib {




    @Override
    public void loop() {
       // arcadedrive();
        holonomicdrive();
        shooter();
        intake();
        magazine();
        wobbleGoal();
        killAll();
        telemetry.update();
    }

    @Override
    public void stop() {
        kill_count = 4;
        killAll();

        super.stop();

    }
}
