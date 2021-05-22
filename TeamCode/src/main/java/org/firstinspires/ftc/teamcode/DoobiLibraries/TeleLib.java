package org.firstinspires.ftc.teamcode.DoobiLibraries;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DoobiLibraries.OdomClasses.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.ThreadHandler;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;

import static android.os.SystemClock.sleep;


public abstract class TeleLib extends OpMode {


    final double COUNTS_PER_INCH = 35;
    final double COUNT_PER_DEGREE = 21;

    double right_stick_x;
    double left_stick_x;
    double left_stick_y;

    double theta;

    boolean arcade = false;
    boolean shooting = false;
    boolean auto_aim = false;


    private DcMotor br;
    private DcMotor bl;
    private DcMotor fr;
    private DcMotor fl;
    private Servo intakemain;
    private Servo intakeclaw;
    private DcMotor shooter;
    private DcMotor pivot;
    private DcMotor lift;
    private DcMotor intake;
    private Servo whook;
    private Servo wobble2;
    private Servo wobble1;
    private Servo mag;
    private Servo pivotStop;
    static double OPEN = 0.0;
    static double CLOSED = 1;
    double liftPower;
    double lift_pos;



    ThreadHandler th_whook;
    ThreadHandler th_wobble;
    ThreadHandler th_lift;
    ThreadHandler th_arcade;
    ThreadHandler th_shooter;
    ThreadHandler th_track;
    ThreadHandler th_holo;

    DcMotor verticalLeft, verticalRight, horizontal;
    String verticalLeftEncoderName = "fr", verticalRightEncoderName = "fl", horizontalEncoderName = "bl";

    OdometryGlobalCoordinatePosition ogcp;

    double wobblePos = .5;
    double hookPos = .5;
    double magPos = 1;
    double position = 1;

    double[] motorPowers;

    public Thread global;
    private boolean magout = false;

    double half;
    boolean halfToggle;

    public Thread gamer_1;
    public Thread gamer_2;

    public double kill_count;
    boolean lift_bottom;
    boolean lift_top;

    boolean track;

    boolean pivot_top;
    boolean pivot_bottom;
    Sensors sensors;
    public TouchSensor button;
    DigitalChannel digitalTouch;



    @Override
    public void init() {
        //Drive base
        sensors = new Sensors(this);
        //Init complet

        br = hardwareMap.dcMotor.get("br");
        bl = hardwareMap.dcMotor.get("bl");
        fr = hardwareMap.dcMotor.get("fr");
        fl = hardwareMap.dcMotor.get("fl");

        pivot = hardwareMap.dcMotor.get("pivot");
        shooter = hardwareMap.dcMotor.get("shooter");
        lift = hardwareMap.dcMotor.get("lift");
        intake = hardwareMap.dcMotor.get("intake");
        //intakemain = hardwareMap.servo.get("intakemain");
        //intakeclaw = hardwareMap.servo.get("intakeclaw");

        whook = hardwareMap.servo.get("whook");
        wobble1 = hardwareMap.servo.get("wobble1");
        wobble2 = hardwareMap.servo.get("wobble2");
        mag = hardwareMap.servo.get("mag");
        pivotStop = hardwareMap.servo.get("ps");
        digitalTouch = hardwareMap.get(DigitalChannel.class, "button");

        // set the digital channel to input.
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);


        wobble1.setDirection(Servo.Direction.FORWARD);
        wobble2.setDirection(Servo.Direction.REVERSE);

        //intake.setDirection(DcMotor.Direction.FORWARD);
        //intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        pivot.setDirection(DcMotor.Direction.REVERSE);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift.setDirection(DcMotor.Direction.FORWARD);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        verticalLeft = fr;
        verticalRight = fl;
        horizontal = bl;

        theta = 0;
        position = 1;

        motorPowers = new double[4];

        resetEncoders();
        resetFlyWheel();

        arcade = true;
        track = false;


        kill_count = 0;
        lift_bottom = true;
        lift_top = false;

        lift_pos = 0;
        liftPower = 0;

        pivot_top = false;
        pivot_bottom = true;

        th_whook = new ThreadHandler();
        th_wobble = new ThreadHandler();
        th_lift = new ThreadHandler();
        th_arcade = new ThreadHandler();
        th_shooter = new ThreadHandler();
        th_track = new ThreadHandler();
        th_holo = new ThreadHandler();

        halfToggle = false;
        half = 1;

        ogcp = new OdometryGlobalCoordinatePosition(fr, fl, bl, COUNTS_PER_INCH, 35);
        Thread global = new Thread(ogcp);
        global.start();

    }


    public void resetFlyWheel(){
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void resetEncoders() {

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    public void drive() {
        if (arcade) {
            arcadedrive();
        } else if(!arcade){
            holonomicdrive();
        }

        telemetry.addData("Drive ", arcade ? "Arcade" : "Holonomic");
    }


    Thread half_speed = new Thread(new Runnable() {
        @Override
        public void run() {
            ElapsedTime time = new ElapsedTime();
            time.reset();
            while(time.milliseconds() < 300){

            }
            half = .5;
            halfToggle = true;
        }
    });

    Thread full_speed = new Thread(new Runnable() {
        @Override
        public void run() {
            ElapsedTime time = new ElapsedTime();
            time.reset();
            while(time.milliseconds() < 300){

            }
            half = 1;
            halfToggle = false;
        }
    });

    public void arcadedrive() {

        if (gamepad1.a && halfToggle == false)
        {
            th_arcade.queue(half_speed);
        }
        else if (gamepad1.a && halfToggle == true)
        {
            th_arcade.queue(full_speed);
        }



        left_stick_y = gamepad1.left_stick_y * half;
        left_stick_x = gamepad1.left_stick_x * half;
        right_stick_x = gamepad1.right_stick_x * half;

        if (Math.abs(left_stick_x) > 0.05 ||
                Math.abs(left_stick_y) > 0.05 ||
                Math.abs(right_stick_x) > 0.05) {

            fl.setPower(left_stick_y + left_stick_x + right_stick_x);
            fr.setPower(left_stick_y - left_stick_x - right_stick_x);
            bl.setPower(left_stick_y - left_stick_x + right_stick_x);
            br.setPower(left_stick_y + left_stick_x - right_stick_x);

        } else {
            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);
        }
        /*
        telemetry.addData("fl encoder", fl.getCurrentPosition());
        telemetry.addData("fr encoder", fr.getCurrentPosition());
        telemetry.addData("bl encoder", bl.getCurrentPosition());
        telemetry.addData("br encoder", br.getCurrentPosition());
        telemetry.addData("halfspeed Thread", th_arcade.live());
        telemetry.addData("Angle", sensors.getGyroYaw());
        //telemetry.addData("TURNING AUTO", auto_turn_aim.isAlive());
        telemetry.addData("Arcade", arcade);

         */

    }


    Thread auto_turn_aim = new Thread(new Runnable() {
        @Override
        public void run() {

            ElapsedTime time = new ElapsedTime();
            while(time.milliseconds() < 300){

            }


            arcade = false;
            right_stick_x = 0;
            double theta = Math.atan2(ogcp.returnXCoordinate(), ogcp.returnYCoordinate());
            double dif_angle = theta - ogcp.returnOrientation();
            while(!arcade){

                right_stick_x = .5 * (dif_angle / 25);
            }

        }
    });


    public void holonomicdrive() {
        left_stick_y = gamepad1.left_stick_y;
        left_stick_x = gamepad1.left_stick_x;
        right_stick_x = gamepad1.right_stick_x;

        theta = -sensors.getGyroYaw();

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

        telemetry.addData("Angle Gyro: ", theta);



    }


    public void killAll(){

        if(gamepad1.right_stick_button){
            kill_count++;
        }

        if(kill_count >= 3){
            pivot.setPower(0);
            lift.setPower(0);
            intake.setPower(0);
            shooter.setPower(0);
            resetEncoders();
            kill_count = 0;
        }
    }


    public void intake() {
        ElapsedTime runtime = new ElapsedTime();
        double right_trigger = gamepad2.right_trigger;
        double left_trigger = gamepad2.left_trigger;
        if (right_trigger > .5) {
            intake.setPower(-right_trigger);
        } else if (left_trigger > .5) {
            intake.setPower(left_trigger);
        } else {
            intake.setPower(0);
        }
    }



    Thread wobble_up = new Thread(new Runnable() {
        @Override
        public void run() {
            ElapsedTime time = new ElapsedTime();
            time.reset();
            while (gamepad2.a && time.milliseconds() < 350) {
            }
            wobble1.setPosition(1);
            wobble2.setPosition(1);
            sleep(700);
        }
    });

    Thread wobble_down = new Thread(new Runnable() {
        @Override
        public void run() {
            ElapsedTime time = new ElapsedTime();
            time.reset();
            while (gamepad2.a && time.milliseconds() < 350) {
            }
            wobble1.setPosition(.2);
            wobble2.setPosition(.2);
            sleep(700);
        }
    });

    Thread whook_open = new Thread(new Runnable() {
        @Override
        public void run() {
            ElapsedTime time = new ElapsedTime();
            time.reset();
            while (time.milliseconds() < 300) {
            }
            whook.setPosition(1);
            sleep(700);
        }
    });

    Thread whook_close = new Thread(new Runnable() {
        @Override
        public void run() {
            ElapsedTime time = new ElapsedTime();
            time.reset();
            while (time.milliseconds() < 300) {
            }
            whook.setPosition(0);
            sleep(700);
        }
    });

    Thread wobble_mid = new Thread(new Runnable() {
        @Override
        public void run() {
            ElapsedTime time = new ElapsedTime();
            time.reset();
            while (time.milliseconds() < 300) {
            }
            wobble1.setPosition(.5);
            wobble2.setPosition(.5);
            sleep(700);
        }
    });



    Thread hardStop_open = new Thread(new Runnable() {
        @Override
        public void run() {
            ElapsedTime time = new ElapsedTime();
            time.reset();
            while (gamepad2.x && time.milliseconds() < 300) {
            }
            pivotStop.setPosition(0);
            sleep(700);
        }
    });
    Thread hardStop_close = new Thread(new Runnable() {
        @Override
        public void run() {
            ElapsedTime time = new ElapsedTime();
            time.reset();
            while (gamepad2.x && time.milliseconds() < 300) {
            }
            pivotStop.setPosition(1);
            sleep(700);
        }
    });

    public void wobbleGoal() {


        if (gamepad2.b && whook.getPosition() != 0) {
            th_whook.queue(whook_close);
        } else if (gamepad2.b && whook.getPosition() == 0) {

            th_whook.queue(whook_open);

        }else if (gamepad2.a && wobble1.getPosition() == 1) {

            th_wobble.queue(wobble_down);

        }else if (gamepad2.a && wobble1.getPosition() !=  1) {

            th_wobble.queue(wobble_up);
        }
        else if (gamepad2.x && pivotStop.getPosition() != 1)
        {
            th_wobble.queue(hardStop_close);
        }

        /*
        telemetry.addData("hook position", whook.getPosition());
        telemetry.addData("wobble position", wobble1.getPosition());
        telemetry.addData("Wobble Thread", th_wobble.live());
        telemetry.addData("Whook Thread", th_whook.live());
        telemetry.addData("pivotStop", pivotStop.getPosition());

         */

    }

    Thread t_shooter_on = new Thread(new Runnable() {
        @Override
        public void run() {
            ElapsedTime time = new ElapsedTime();
            while(time.milliseconds() < 300){

            }
            shooter.setPower(.9);
            shooting = true;
        }
    });
    Thread t_shooter_off = new Thread(new Runnable() {
        @Override
        public void run() {
            ElapsedTime time = new ElapsedTime();
            while(time.milliseconds() < 300){

            }
            shooter.setPower(0);
            shooting = false;
        }
    });


    Thread track_th = new Thread(new Runnable() {
        @Override
        public void run() {
            ElapsedTime time = new ElapsedTime();
            double initial = shooter.getCurrentPosition();
            double velocity = 0;
            time.reset();
            while(track){
                velocity = (shooter.getCurrentPosition() - initial)/(time.seconds());
                telemetry.addLine("WHEEL IS ON");
                telemetry.addData("VELOCITY : ", velocity);
                telemetry.update();
            }
        }
    });

    Thread auto_pivot_up = new Thread(new Runnable() {
        @Override
        public void run() {

            ElapsedTime time_s = new ElapsedTime();
            while(time_s.milliseconds() < 300){

            }
            auto_aim = true;

            while(auto_aim){
                double x = ogcp.returnXCoordinate();
                double y = ogcp.returnYCoordinate();
                double distance = Math.sqrt(x*x + y*y);
                double theta = Shooter.calcThetaPivot(distance, 1, 100);
                theta = 45;
                double pos = theta * COUNT_PER_DEGREE;

                if(theta > 50){
                    return;
                }
                double timeout = 2;

                boolean moveUp = true;
                double kP = .07/pos;
                double kI = .01;
                double kD = .01/pos;

                ElapsedTime time = new ElapsedTime();
                ElapsedTime timeoutTimer = new ElapsedTime();

                double error;
                double power;

                double proportional;
                double integral = 0;
                double derivative;

                double prevRunTime;

                double initPos = pivot.getCurrentPosition();

                double lastError = pos - initPos;

                time.reset();
                timeoutTimer.reset();
                while (Math.abs(pos - pivot.getCurrentPosition()) > 10 && timeoutTimer.seconds() < timeout && pos < 1100) {
                    prevRunTime = time.seconds();

                    error = pos - pivot.getCurrentPosition();

                    proportional = error * kP;

                    integral += (error * (time.seconds() - prevRunTime)) * kI;


                    derivative = ((error - lastError) / (time.seconds() - prevRunTime)) * kD;


                    power = proportional + integral + derivative;

                    if (moveUp)
                    {
                        pivot.setPower(power);
                    }
                    else
                    {
                        pivot.setPower(-power);
                    }

                /*
                telemetry.addData("error ", error);
                telemetry.addData("P", proportional);
                telemetry.addData("I", integral);
                telemetry.addData("D", derivative);
                telemetry.addData("power", power);
                telemetry.update();
                */
                    lastError = error;

                    if (Math.abs(pos - pivot.getCurrentPosition()) < 10)
                    {
                        break;
                    }

                }
                telemetry.addLine("exited");
                telemetry.update();

                pivot.setPower(0);
            }

        }
    });


    public void odom_shooter(){
        if (gamepad2.right_bumper && !shooting) {
            track = true;
            th_track.queue(track_th);
            th_shooter.queue(t_shooter_on);
        } else if(gamepad2.right_bumper && shooting){
            track = false;
            th_track.th_kill();
            th_shooter.queue(t_shooter_off);
        }


        double pivotPos = pivot.getCurrentPosition();
        if(gamepad2.dpad_up && lift_top && pivotPos < 1100){

            th_lift.queue(auto_pivot_up);
        } else if (gamepad2.dpad_down && lift_top && pivotPos > -700) {
            pivot.setPower(-.5);
        } else if (gamepad2.dpad_right && lift_top && pivotPos < 1200) {
            pivot.setPower(.25);
        } else if (gamepad2.dpad_left && lift_top && pivotPos > -700) {
            pivot.setPower(-.05);
        } else {
            pivot.setPower(.05);
        }
        telemetry.addData("pivot encoder pos: ", pivot.getCurrentPosition());
        telemetry.addData("lift encoder pos: ", lift.getCurrentPosition());

    }

    public void shooter() {
        if (gamepad2.right_bumper && !shooting) {
            track = true;
            th_track.queue(track_th);
            th_shooter.queue(t_shooter_on);
        } else if(gamepad2.right_bumper && shooting){
            track = false;
            th_track.th_kill();
            th_shooter.queue(t_shooter_off);
        }


        double pivotPos = pivot.getCurrentPosition();
        if (gamepad2.dpad_up && lift_top && pivotPos < 1100) {
            pivot.setPower(.5);

        } else if (gamepad2.dpad_down && lift_top && pivotPos > -700) {
            pivot.setPower(-.5);
        } else if (gamepad2.dpad_right && lift_top && pivotPos < 1100) {
            pivot.setPower(.25);
        } else if (gamepad2.dpad_left && lift_top && pivotPos > -700) {
            pivot.setPower(-.05);
        } else {
            pivot.setPower(.05);
        }

        telemetry.addData("pivot encoder pos: ", pivot.getCurrentPosition());
        telemetry.addData("lift encoder pos: ", lift.getCurrentPosition());


    }


    Thread lift_up = new Thread(new Runnable() {
        @Override
        public void run() {
            lift.setPower(.6);
            ElapsedTime time = new ElapsedTime();

            time.reset();
            while(lift.getCurrentPosition() < 190 && time.milliseconds() < 500){

            }
            //sleep(700);
            lift.setPower(.4);
            lift_top = true;
            lift_bottom = false;
            sleep(300);
        }
    });

    Thread lift_down = new Thread(new Runnable() {
        @Override
        public void run() {
            lift.setPower(-.3);
            sleep(500);
            lift.setPower(-.2);
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lift_bottom = true;
            lift_top = false;
            sleep(300);
        }
    });

    Thread mag_thread= new Thread(new Runnable() {
        @Override
        public void run() {
            mag.setPosition(0);
            sleep(150);
            mag.setPosition(1);
        }
    });

    public void magazine() {
        double right_stick_y = -gamepad2.right_stick_y;
        if (right_stick_y > .1 && !lift_top) {

            th_lift.queue(lift_up);

        } else if (right_stick_y < -.1 && !lift_bottom) {

            th_lift.queue(lift_down);
        }


        if (gamepad2.left_bumper && !magout) {

            mag_thread.start();
        }

        telemetry.addData("Lift Threading Up : ", lift_up.isAlive());
        telemetry.addData("Lift Threading Down : ", lift_down.isAlive());

    }


    @Override
    public void stop() {
        super.stop();
        track = false;
        arcade = true;
    }
}
