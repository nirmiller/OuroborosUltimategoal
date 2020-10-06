package org.firstinspires.ftc.teamcode.DoobiLibraries.OdomClasses;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class OdomDriveTrain {
    //Drive motors
    DcMotor right_front, right_back, left_front, left_back;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;


    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String rfName = "fr", rbName = "br", lfName = "fl", lbName = "bl";
    String verticalLeftEncoderName = "bl", verticalRightEncoderName = "br", horizontalEncoderName = "fl";

    double theta;

    final double COUNTS_PER_INCH = 308.876;

    LinearOpMode linearOpMode;

    public OdomDriveTrain(LinearOpMode linearOpMode)
    {
        this.linearOpMode = linearOpMode;
        right_front = linearOpMode.hardwareMap.dcMotor.get(rfName);
        right_back = linearOpMode.hardwareMap.dcMotor.get(rbName);
        left_front = linearOpMode.hardwareMap.dcMotor.get(lfName);
        left_back = linearOpMode.hardwareMap.dcMotor.get(lbName);

        verticalLeft = linearOpMode.hardwareMap.dcMotor.get(verticalLeftEncoderName);
        verticalRight = linearOpMode.hardwareMap.dcMotor.get(verticalRightEncoderName);
        horizontal = linearOpMode.hardwareMap.dcMotor.get(horizontalEncoderName);

        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_front.setDirection(DcMotor.Direction.FORWARD);
        right_front.setDirection(DcMotor.Direction.REVERSE);
        left_back.setDirection(DcMotor.Direction.FORWARD);
        right_back.setDirection(DcMotor.Direction.REVERSE);

        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_back.setDirection(DcMotorSimple.Direction.REVERSE);

        linearOpMode.telemetry.addData("Status", "Hardware Map Init Complete");
        linearOpMode.telemetry.update();
    }
    public void resetEncoders() {

        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }


    public void choop()
    {
        left_front.setPower(0);
        right_front.setPower(0);
        left_back.setPower(0);
        right_back.setPower(0);
    }



}
