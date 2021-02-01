package org.firstinspires.ftc.teamcode.DoobiOps;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.DoobiLibraries.TeleLib;



@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleDrive")
public class TeleOp extends TeleLib {




    @Override
    public void loop() {
        arcadedrive();
        magazine();
        shooter();
        wobbleGoal();
        intake();
        telemetry.update();
    }

    @Override
    public void stop() {


        gamer_1.interrupt();
        gamer_2.interrupt();
        global.interrupt();
        super.stop();

    }
}
