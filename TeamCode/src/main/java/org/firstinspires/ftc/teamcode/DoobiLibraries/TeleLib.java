package org.firstinspires.ftc.teamcode.DoobiLibraries;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.DoobiLibraries.OdomClasses.OdometryGlobalCoordinatePosition;


public abstract class TeleLib extends OpMode {

    final double COUNTS_PER_INCH = 308.876;

    double right_stick_x;
    double left_stick_x;
    double left_stick_y;

    double theta;

    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;

    double[] motorPowers;

    OdometryGlobalCoordinatePosition ogcp;

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


        theta = 0;

        motorPowers = new double[4];

        resetEncoders();

        ogcp = new OdometryGlobalCoordinatePosition(bl, br, fl, COUNTS_PER_INCH, 10);
        ogcp.run();


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
        theta = Math.toRadians(ogcp.returnOrientation());

        if (Math.abs(left_stick_x) > 0.05 ||
                Math.abs(left_stick_y) > 0.05 ||
                Math.abs(right_stick_x) > 0.05) {

            motorPowers = Holonomic.calcPowerTele(theta, right_stick_x, left_stick_x, left_stick_y);

            fl.setPower(motorPowers[0]);
            fr.setPower(motorPowers[1]);
            bl.setPower(motorPowers[2]);
            br.setPower(motorPowers[3]);

        } else {
            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);
        }

        telemetry.addData("Angle : ", ogcp.returnOrientation());

    }

}
