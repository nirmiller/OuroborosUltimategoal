package org.firstinspires.ftc.teamcode.DoobiLibraries;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.DoobiLibraries.OdomClasses.OdometryGlobalCoordinatePosition;


public abstract class TeleLib extends OpMode {


    final double COUNTS_PER_INCH = 308.876;

    double right_stick_x;
    double left_stick_x;
    double left_stick_y;

    double theta;

    boolean arcade = false;


    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;
    private DcMotor intake;
    private DcMotor shooter;
    private DcMotor pivot;
    private DcMotor lift;
    private Servo hook;
    private Servo wobble;
    private Servo mag;

    DcMotor verticalLeft, verticalRight, horizontal;
    String verticalLeftEncoderName = "bl", verticalRightEncoderName = "br", horizontalEncoderName = "fl";

    int wobblePos = 1;
    int hookPos = 1;
    int magPos = 1;

    double[] motorPowers;

    public OdometryGlobalCoordinatePosition ogcp;
    public Thread global;
    @Override
    public void init() {
        //Drive base

        //Init complete
        telemetry.addData("Status", "Init Complete");
        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");
        pivot = hardwareMap.dcMotor.get("pivot");
        shooter = hardwareMap.dcMotor.get("shooter");
        lift = hardwareMap.dcMotor.get("lift");
        hook = hardwareMap.servo.get("whook");
        wobble = hardwareMap.servo.get("wobble");
        mag = hardwareMap.servo.get("mag");

        fl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        verticalLeft = bl;
        verticalRight = br;
        horizontal = fl;

        theta = 0;

        motorPowers = new double[4];

        resetEncoders();

        ogcp = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 35);
        global = new Thread(ogcp);
        global.start();
        arcade = false;


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


    public void drive()
    {
        if(arcade && gamepad1.b)
        {
            arcade = false;
        }else if (!arcade && gamepad1.b)
        {
            arcade = true;
        }
        if(arcade)
        {
            arcadedrive();
        }else {
            holonomicdrive();
        }

        telemetry.addData("Drive ", arcade ? "Arcade" : "Holonomic");
    }


    public void arcadedrive(){
        left_stick_y = gamepad1.left_stick_y;
        left_stick_x = gamepad1.left_stick_x;
        right_stick_x = gamepad1.right_stick_x;

        if (Math.abs(left_stick_x) > 0.05 ||
                Math.abs(left_stick_y) > 0.05 ||
                Math.abs(right_stick_x) > 0.05) {

            fl.setPower(left_stick_y + left_stick_x - right_stick_x);
            fr.setPower(left_stick_y - left_stick_x + right_stick_x);
            bl.setPower(left_stick_y - left_stick_x - right_stick_x);
            br.setPower(left_stick_y + left_stick_x + right_stick_x);

        }else {
            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);
        }


        }

    public void holonomicdrive() {
        left_stick_y = -gamepad1.left_stick_y;
        left_stick_x = gamepad1.left_stick_x;
        right_stick_x = gamepad1.right_stick_x;
        theta = ogcp.returnOrientation();


        double[] motors = new double[4];
        motors = Holonomic.calcPowerTele(theta, right_stick_x, left_stick_x, left_stick_y);

        if (Math.abs(left_stick_x) > 0.05 ||
                Math.abs(left_stick_y) > 0.05 ||
                Math.abs(right_stick_x) > 0.05) {

            motors = Holonomic.calcPowerTele(theta, right_stick_x, left_stick_x, left_stick_y);

            fl.setPower(motors[0]);
            fr.setPower(motors[1]);
            bl.setPower(motors[2]);
            br.setPower(motors[3]);

        } else {
            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);
        }

        telemetry.addData("Angle : ", ogcp.returnOrientation());
        telemetry.addData("X Position : ", ogcp.returnXCoordinate());
        telemetry.addData("Y Position : ", ogcp.returnYCoordinate());

    }
    public void wobbleGoal(){
        if (gamepad2.a) {
            hook.setPosition(hookPos);
            hookPos = Math.abs(hookPos - 1);
        }

        if (gamepad2.b) {
            wobble.setPosition(wobblePos);
            wobblePos = Math.abs(wobblePos - 1);
        }
        telemetry.addData("hook position", hook.getPosition());
        telemetry.addData("wobble position", wobble.getPosition());
    }
    public void output(){
        if (Math.abs(gamepad2.left_stick_y) > .05){
            shooter.setPower(gamepad2.left_stick_y);
        }
        if (gamepad2.right_bumper){
            mag.setPosition(magPos);
            magPos = Math.abs(magPos - 1);
        }
    }

}
