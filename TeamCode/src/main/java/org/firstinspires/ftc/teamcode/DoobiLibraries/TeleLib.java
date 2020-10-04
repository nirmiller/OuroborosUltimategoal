package org.firstinspires.ftc.teamcode.DoobiLibraries;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


public abstract class TeleLib extends OpMode {

    final double COUNTS_PER_INCH = 308.876;

    double right_stick_x;
    double left_stick_x;
    double left_stick_y;

    double odomRight;
    double odomLeft;
    double odomPerp;

    double theta;

    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;

    double X_i;
    double deltaX;

    double Y_i;
    double deltaY;

    @Override
    public void init() {
        //Drive base

        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");

        fl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        odomPerp = 0;
        odomRight = 0;
        odomLeft = 0;
        theta = 0;

        X_i = 0;
        Y_i = 0;


        resetEncoders();
    }

    public void resetEncoders() {

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }



    public void arcadedrive() {
        left_stick_y = gamepad1.left_stick_y;
        left_stick_x = gamepad1.left_stick_x;
        right_stick_x = gamepad1.right_stick_x;

        odomLeft = -bl.getCurrentPosition() / COUNTS_PER_INCH;
        odomRight = -br.getCurrentPosition() / COUNTS_PER_INCH;
        odomPerp = fl.getCurrentPosition() / COUNTS_PER_INCH ;

        theta = ((odomLeft - odomRight)/8.25) % (2 * Math.PI);


        if (Math.abs(left_stick_x) > 0.05 ||
                Math.abs(left_stick_y) > 0.05 ||
                Math.abs(right_stick_x) > 0.05) {

            double x_comp = left_stick_y*Math.cos(theta) + left_stick_x*Math.sin(theta);
            double y_comp = left_stick_y*Math.sin(theta) - left_stick_x*Math.cos(theta);
            double rot_comp = right_stick_x;

            fl.setPower(x_comp - y_comp - .5*rot_comp);
            fr.setPower(x_comp + y_comp + .5*rot_comp);
            bl.setPower(x_comp + y_comp - .5*rot_comp);
            br.setPower(x_comp - y_comp + .5*rot_comp);

        } else {
            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);
        }

        telemetry.addData("Angle : ", Math.toDegrees(theta));

    }


}
