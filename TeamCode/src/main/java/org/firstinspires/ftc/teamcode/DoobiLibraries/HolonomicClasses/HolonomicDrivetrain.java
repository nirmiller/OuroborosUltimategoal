package org.firstinspires.ftc.teamcode.DoobiLibraries.HolonomicClasses;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DoobiLibraries.Holonomic;
import org.firstinspires.ftc.teamcode.DoobiLibraries.OdomClasses.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.DoobiLibraries.Point;
import org.firstinspires.ftc.teamcode.DoobiLibraries.Sensors;

import java.util.ArrayList;
import java.util.concurrent.CountDownLatch;

public class HolonomicDrivetrain {

    //Drive motors
    DcMotor fl, fr, bl, br;
    Servo hook;
    Servo wobble;
    Servo mag;

    public Thread global;

    double theta;

    final double COUNTS_PER_INCH = 35;

    LinearOpMode opMode;
    Sensors sensors;

    public HolonomicDrivetrain(LinearOpMode opMode) {
        this.opMode = opMode;
        sensors = new Sensors(opMode);

        fl = opMode.hardwareMap.dcMotor.get("fl");
        fr = opMode.hardwareMap.dcMotor.get("fr");
        bl = opMode.hardwareMap.dcMotor.get("bl");
        br = opMode.hardwareMap.dcMotor.get("br");


        //intake = hardwareMap.dcMotor.get("intake");

        hook = opMode.hardwareMap.servo.get("whook");
        //wobble = opMode.hardwareMap.servo.get("wobble");
        mag = opMode.hardwareMap.servo.get("mag");


        //intake = hardwareMap.dcMotor.get("intake");


        mag = opMode.hardwareMap.servo.get("mag");

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);

        resetEncoders();

        sensors = new Sensors(opMode);

        opMode.telemetry.addData("Status", "Hardware Map Init Complete");
        opMode.telemetry.update();
    }

    public void end() {
        choop();
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


    public void goToPoint(double targetX, double targetY, double face, double power, double allowedDistanceError, double timeout) {

        ElapsedTime time = new ElapsedTime();
        time.reset();

        //distance to x and y for trig calculations
        double distanceToX = targetX;
        double distanceToY = targetY;

        //gets total distance needed to travel
        double distance = Math.hypot(distanceToX, distanceToY);
        double[] motor = new double[4];
        double angle_heading = Math.toDegrees(Math.atan2(distanceToX, distanceToY));
        double position_x = 0;
        double position_y = 0;
        double[] positions = new double[4];
        //0:fl, 1:fr, 2:bl, 3:br
        double angle_correction = face;
        double rot_power = 0;
        while (opMode.opModeIsActive() && distance > allowedDistanceError && time.seconds() < timeout) {

            position_x += position_i(angle_heading, sensors.getGyroYawwwwwwwwwwwwwwwwwww(), fl.getCurrentPosition() - positions[0],
                    fr.getCurrentPosition() - positions[1], bl.getCurrentPosition() - positions[2],
                    br.getCurrentPosition() - positions[3])[0];
            position_y += position_i(angle_heading, sensors.getGyroYawwwwwwwwwwwwwwwwwww(), fl.getCurrentPosition() - positions[0],
                    fr.getCurrentPosition() - positions[1], bl.getCurrentPosition() - positions[2],
                    br.getCurrentPosition() - positions[3])[1];


            distanceToX = targetX - position_x;
            distanceToY = targetY - position_y;
            distance = Math.hypot(distanceToX, distanceToY);
            angle_correction = sensors.getGyroYawwwwwwwwwwwwwwwwwww() - face;

            motor = Holonomic.calcPowerAuto(angle_heading, angle_correction, rot_power);


            fl.setPower(motor[0] * power);
            fr.setPower(motor[1] * power);
            bl.setPower(motor[2] * power);
            br.setPower(motor[3] * power);


            opMode.telemetry.addData("ANGLE", angle_correction);
            opMode.telemetry.update();
            if (angle_correction >= 1) {
                rot_power = Math.cos(angle_heading) * Math.abs(angle_correction) / 15;
            } else if (angle_correction <= -1) {
                rot_power = Math.cos(angle_heading) * -Math.abs(angle_correction) / 15;
            } else {
                rot_power = 0;
            }

            //figures out what power to set the motors to so we can move at this angle
            opMode.telemetry.addData("Angle : ", sensors.getGyroYaw());
            opMode.telemetry.addData("X Position : ", position_x);
            opMode.telemetry.addData("Y Position : ", position_y);
            opMode.telemetry.update();

            positions[0] = fl.getCurrentPosition();
            positions[1] = fr.getCurrentPosition();
            positions[2] = bl.getCurrentPosition();
            positions[3] = br.getCurrentPosition();


        }

        //choop();
    }
/*
    public void splineMove(ArrayList<Point> spline, double power, double timeout, double allowed_error) {
        for (Point p : spline) {
            goToPoint(p.getX(), p.getY(), 0, power, allowed_error, timeout);
        }
        choop();
    }
*/

    public void turnPID(double angleChange, boolean turnRight, double kP, double kI, double kD, double timeout) {

        ElapsedTime time = new ElapsedTime();
        ElapsedTime timeoutTimer = new ElapsedTime();

        double error;
        double power;

        double proportional;
        double integral = 0;
        double derivative;

        double prevRunTime;

        double initAngle = sensors.getGyroYaw();

        double lastError = angleChange - Math.abs(sensors.getGyroYaw() - initAngle);

        time.reset();
        timeoutTimer.reset();

        double control = .4;

        while (Math.abs(sensors.getGyroYaw() - (angleChange + initAngle)) > .1 && timeoutTimer.seconds() < timeout && opMode.opModeIsActive()) {
            prevRunTime = time.seconds();


            error = angleChange - Math.abs(sensors.getGyroYaw() - initAngle);


            proportional = error * kP;

            integral += (error * (time.seconds() - prevRunTime)) * kI;


            derivative = ((error - lastError) / (time.seconds() - prevRunTime)) * kD;


            power = (proportional + integral + derivative)*control;

            if (power < .3 && kI == 0 && kD == 0) {
                power = .25;
            }

            if(control < 1){
                control += .01;
            }

            turn(power, turnRight);

            opMode.telemetry.addData("error ", error);
            opMode.telemetry.addData("P", proportional);
            opMode.telemetry.addData("I", integral);
            opMode.telemetry.addData("D", derivative);
            opMode.telemetry.addData("power", power);
            opMode.telemetry.update();

            lastError = error;
           /* if (sensors.getGyroYaw2() < 360 && sensors.getGyroYaw() > 180)
            {
                initAngle = 360;
            }*/

            opMode.idle();


        }
        opMode.telemetry.addLine("exited");
        opMode.telemetry.update();

        choop();

    }

    public void turn(double power, boolean isRight) {
        if (isRight) {
            fr.setPower(-power);
            br.setPower(-power);
            fl.setPower(power);
            bl.setPower(power);
        } else {
            fr.setPower(power);
            br.setPower(power);
            fl.setPower(-power);
            bl.setPower(-power);
        }
    }


    public double getEncoderAverage() {

        double count = 2.0;
        if (fr.getCurrentPosition() == 0) {
            count--;
        }
        if (fl.getCurrentPosition() == 0) {
            count--;
        }
        if (br.getCurrentPosition() == 0) {
            count--;
        }
        if (bl.getCurrentPosition() == 0) {
            count--;
        }
        if (count == 0) {
            return 0;
        }
        return (fl.getCurrentPosition() + fr.getCurrentPosition()) / count;
    }

    private double getStrafeEncoderAverage(double direction) {

        double count = 4.0;
        double average = 0;

        if (fr.getCurrentPosition() == 0) {
            count--;
        }
        if (fl.getCurrentPosition() == 0) {
            count--;
        }
        if (br.getCurrentPosition() == 0) {
            count--;
        }
        if (bl.getCurrentPosition() == 0) {
            count--;
        }
        if (count == 0) {
            return 0;
        }
        if (direction < 0) {
            average = (((-fl.getCurrentPosition() + -1 * fr.getCurrentPosition()
                    + -br.getCurrentPosition() + bl.getCurrentPosition()))) / count;
        } else if (direction > 0) {
            average = (((fl.getCurrentPosition() + -fr.getCurrentPosition()
                    + br.getCurrentPosition() + -1 * -bl.getCurrentPosition()))) / count;
        }
        return average;
    }

    public void setMotorsPower(double power) {

        fl.setPower(power);
        fr.setPower(power);
        br.setPower(power);
        bl.setPower(power);
    }

    public void setStrafePower(double power) {
        fr.setPower(power);
        br.setPower(-power);
        fl.setPower(-power);
        bl.setPower(power);
    }

    public void encoderMove(double power, double distance, double runtimeS) {
        resetEncoders();
        ElapsedTime time = new ElapsedTime();

        double initEncoder = 0;

        time.reset();

        distance = distance * COUNTS_PER_INCH;
        double finalPower = power;
        power = .2;
        double d = 0;
        while (d < distance && time.seconds() < runtimeS && opMode.opModeIsActive()) {
            d = Math.abs(getEncoderAverage() - initEncoder);
            if (d <= distance / 1.25) {
                power += .05;
            } else if (d > distance / 1.25) {
                power += -.025;
            }
            setMotorsPower(power);
            if (!opMode.opModeIsActive()) {
                choop();
            }

            opMode.telemetry.addData("Encoder distance left", (distance - getEncoderAverage()));
            opMode.telemetry.update();

        }
        choop();
    }

    public void gyroTurnNinety(double timeOutMS) {

        ElapsedTime runtime = new ElapsedTime();
        double goal = 90;

        do {

            opMode.telemetry.addData("Goal", goal);
            opMode.telemetry.addData("Current Heading", sensors.getGyroYaw());
            opMode.telemetry.update();
            if (sensors.getGyroYaw() < goal) {
                turn(.21, false);
            } else {
                turn(.21, true);
            }


        } while (opMode.opModeIsActive() && Math.abs(goal - sensors.getGyroYaw()) > 2 && runtime.milliseconds() < timeOutMS);

        choop();
    }


    public void gyroTurnNinetyFast(double timeOutMS) {

        ElapsedTime runtime = new ElapsedTime();
        double goal = 90;

        do {

            opMode.telemetry.addData("Goal", goal);
            opMode.telemetry.addData("Current Heading", sensors.getGyroYaw());
            opMode.telemetry.update();
            if (sensors.getGyroYaw() < goal && sensors.getGyroYaw() > 0) {
                turn(.7, false);
            } else {
                turn(.7, true);
            }


        } while (opMode.opModeIsActive() && Math.abs(goal - sensors.getGyroYaw()) > 2 && runtime.milliseconds() < timeOutMS);

        choop();
    }

    public void gyroTurn270(double timeOutMS) {

        ElapsedTime runtime = new ElapsedTime();
        double goal = 270;

        do {

            opMode.telemetry.addData("Goal", goal);
            opMode.telemetry.addData("Current Heading", sensors.getGyroYaw());
            opMode.telemetry.update();
            if (sensors.getGyroYaw() < goal) {
                turn(.27, false);
            } else {
                turn(.27, true);
            }


        } while (opMode.opModeIsActive() && Math.abs(goal - sensors.getGyroYaw()) > .05 && runtime.milliseconds() < timeOutMS);

        choop();
    }

    public void gyroTurn270Fast(double timeOutMS) {

        ElapsedTime runtime = new ElapsedTime();
        double goal = 270;

        do {

            opMode.telemetry.addData("Goal", goal);
            opMode.telemetry.addData("Current Heading", sensors.getGyroYaw());
            opMode.telemetry.update();

            turn(.7, true);


        } while (opMode.opModeIsActive() && Math.abs(goal - sensors.getGyroYaw()) > 2 && runtime.milliseconds() < timeOutMS);

        choop();
    }

    public void gyroTurn180(double timeOutMS) {

        ElapsedTime runtime = new ElapsedTime();
        double goal = 182;


        while (opMode.opModeIsActive() && Math.abs(goal - sensors.getGyroYaw()) > .1 && runtime.milliseconds() < timeOutMS) {

            opMode.telemetry.addData("Goal", goal);
            opMode.telemetry.addData("Current Heading", sensors.getGyroYaw());
            opMode.telemetry.update();
            if (sensors.getGyroYaw() < goal) {
                turn(.25, false);
            } else {
                turn(.25, true);
            }


        }

        choop();
    }

    public void gyroTurn180Red(double timeOutMS) {

        ElapsedTime runtime = new ElapsedTime();
        double goal = 192;


        while (opMode.opModeIsActive() && Math.abs(goal - sensors.getGyroYaw()) > .1 && runtime.milliseconds() < timeOutMS) {

            opMode.telemetry.addData("Goal", goal);
            opMode.telemetry.addData("Current Heading", sensors.getGyroYaw());
            opMode.telemetry.update();
            if (sensors.getGyroYaw() < goal) {
                turn(.25, false);
            } else {
                turn(.25, true);
            }


        }

        choop();
    }

    public void gyroTurn180FastRed(double timeOutMS) {

        ElapsedTime runtime = new ElapsedTime();
        double goal = 186;
        if (sensors.getGyroYaw() > 0 && sensors.getGyroYaw() < 180) {
            goal = 190;
        } else {
            goal = 190;
        }


        do {

            opMode.telemetry.addData("Goal", goal);
            opMode.telemetry.addData("Current Heading", sensors.getGyroYaw());
            opMode.telemetry.update();
            if (sensors.getGyroYaw() < goal && sensors.getGyroYaw() > 0) {
                turn(.7, false);
            } else {
                turn(.7, true);
            }


        } while (opMode.opModeIsActive() && Math.abs(goal - sensors.getGyroYaw()) > 2 && runtime.milliseconds() < timeOutMS);

        choop();
    }

    public void gyroTurn180Fast(double timeOutMS) {

        ElapsedTime runtime = new ElapsedTime();
        double goal = 180;
        if (sensors.getGyroYaw() > 0 && sensors.getGyroYaw() < 180) {
            goal = 182;
        } else {
            goal = 182;
        }


        do {

            opMode.telemetry.addData("Goal", goal);
            opMode.telemetry.addData("Current Heading", sensors.getGyroYaw());
            opMode.telemetry.update();
            if (sensors.getGyroYaw() < goal && sensors.getGyroYaw() > 0) {
                turn(.7, false);
            } else {
                turn(.7, true);
            }


        } while (opMode.opModeIsActive() && Math.abs(goal - sensors.getGyroYaw()) > 2 && runtime.milliseconds() < timeOutMS);

        choop();
    }

    public void gyroTurnStraight(double timeOutMS) {

        ElapsedTime runtime = new ElapsedTime();
        double goal = 0;

        do {

            if (sensors.getGyroYaw() > 0 && sensors.getGyroYaw() < 180) {
                goal = 0;
            } else {
                goal = 360;
            }

            opMode.telemetry.addData("Goal", goal);
            opMode.telemetry.addData("Current Heading", sensors.getGyroYaw());
            opMode.telemetry.update();
            if (sensors.getGyroYaw() < goal) {
                turn(.3, false);
            } else {
                turn(.3, true);
            }


        } while (opMode.opModeIsActive() && Math.abs(goal - sensors.getGyroYaw()) != 0 && runtime.milliseconds() < timeOutMS);

        choop();
    }

    public void gyroTurnStraightfast(double timeOutMS) {

        ElapsedTime runtime = new ElapsedTime();
        double goal = 0;

        while (opMode.opModeIsActive() && Math.abs(goal - sensors.getGyroYaw()) > .1 && runtime.milliseconds() < timeOutMS) {

            if (sensors.getGyroYaw() > 0 && sensors.getGyroYaw() < 180) {
                goal = 0;
            } else {
                goal = 360;
            }

            opMode.telemetry.addData("Goal", goal);
            opMode.telemetry.addData("Current Heading", sensors.getGyroYaw());
            opMode.telemetry.update();
            if (sensors.getGyroYaw() < goal) {
                turn(.7, false);
            } else {
                turn(.7, true);
            }


        }
        while (opMode.opModeIsActive() && Math.abs(goal - sensors.getGyroYaw()) > 2 && runtime.milliseconds() < timeOutMS)
            ;

        choop();
    }

    public void gyroHoloStrafe(double power, double distance, boolean left, double timeoutMS, double initial) {
        distance = distance * COUNTS_PER_INCH;
        ElapsedTime time = new ElapsedTime();
        resetEncoders();
        double count = 0;

        double[] motor_power = new double[4];

        double average = 0;
        double angle_heading = 0;
        double angle_face = 0;
        double rot_power = 0;
        resetEncoders();

        power = power * .6;

        double pos = 1;
        if (left) {
            pos = -1;
            angle_heading = -90;
        } else {
            pos = 1;
            angle_heading = 90;
        }


        while (opMode.opModeIsActive() && (distance) - Math.abs(getStrafeEncoderAverage(pos)) > 0 && time.milliseconds() < timeoutMS) {
            average = getEncoderAverage();

            angle_face = sensors.getGyroYawwwwwwwwwwwwwwwwwww() - initial;


            if (Math.abs(power) >= .2 && distance - Math.abs(getEncoderAverage()) > distance / 7) {
                power += .05;
            } else {
                power -= .01;
            }

            motor_power = Holonomic.calcPowerAuto(angle_heading, angle_face, rot_power);
            fl.setPower(motor_power[0] * power);
            fr.setPower(motor_power[1] * power);
            bl.setPower(motor_power[2] * power);
            br.setPower(motor_power[3] * power);
            opMode.telemetry.addData("ANGLE", angle_face);
            opMode.telemetry.update();
            if (angle_face >= 1) {
                rot_power = Math.abs(angle_face) / 10;
            } else if (angle_face <= -1) {
                rot_power = -Math.abs(angle_face) / 10;
            } else {
                rot_power = 0;
            }

        }
        gyroTurnStraight(500);

        choop();
    }
//k

    public void gyroHoloForward(double power, double distance, double timeoutMS, double initial) {
        distance = distance * COUNTS_PER_INCH;
        ElapsedTime time = new ElapsedTime();
        double count = 0;

        double[] motor_power = new double[4];

        double average = 0;
        double angle_heading = 0;
        double angle_face = 0;
        double rot_power = 0;
        //resetEncoders();
        double init_distance = getEncoderAverage();

        power = power * .5;
        double pos = power / Math.abs(power);
        while (opMode.opModeIsActive() && (distance) - Math.abs(getEncoderAverage() - init_distance) > .5 && time.milliseconds() < timeoutMS) {
            average = getEncoderAverage();

            angle_face = sensors.getGyroYawwwwwwwwwwwwwwwwwww() - initial;


            if (Math.abs(power) >= .2 && distance - Math.abs(getEncoderAverage() - init_distance) > distance / 4) {
                power += .03 * pos;
            } else {
                power -= .03 * pos;
            }


            motor_power = Holonomic.calcPowerAuto(angle_heading, angle_face, rot_power);
            fl.setPower(motor_power[0] * power);
            fr.setPower(motor_power[1] * power);
            bl.setPower(motor_power[2] * power);
            br.setPower(motor_power[3] * power);
            opMode.telemetry.addData("ANGLE", angle_face);
            opMode.telemetry.update();
            if (angle_face >= .5) {
                rot_power = pos * Math.abs(angle_face) / 30;
            } else if (angle_face <= -.5) {
                rot_power = pos * -Math.abs(angle_face) / 30;
            } else {
                rot_power = 0;
            }

        }
        choop();
    }


    public void choop() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    /**
     * Calculate the power in the x direction
     *
     * @param desiredAngle angle on the x axis
     * @param speed        robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     *
     * @param desiredAngle angle on the y axis
     * @param speed        robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }

    //pos[1] = y
    //pos[0] = x

    public double[] position_i(double angle_heading, double angle_face, double fl_pos, double fr_pos, double bl_pos, double br_pos) {
        double[] pos = new double[2];

        angle_heading = Math.toRadians(angle_heading);
        angle_face = Math.toRadians(angle_face);

        pos[1] = ((Math.cos(angle_face - angle_heading)) * (fl_pos + fr_pos + bl_pos + br_pos)) / (COUNTS_PER_INCH);
        pos[0] = ((Math.sin(angle_face - angle_heading)) * (fl_pos - fr_pos - bl_pos + br_pos)) / (COUNTS_PER_INCH);
        return pos;
    }


    public void gyroHoloPIDMovement(double heading, double face, double distance, double timeoutS, double kP, double kI, double kD) {
        ElapsedTime time = new ElapsedTime();
        ElapsedTime timeoutTimer = new ElapsedTime();

        double power;

        double angle_face = 0;
        double rot_power = 0;

        double proportional;
        double integral = 0;
        double derivative;

        double[] motor_power = new double[4];

        double prevRunTime;

        double initial = getEncoderAverage();

        double average = Math.abs((getEncoderAverage() - initial)/ COUNTS_PER_INCH);

        double error = distance - average;
        double lastError = error;


        time.reset();
        timeoutTimer.reset();

        while (Math.abs(error) > 1 && timeoutTimer.seconds() < timeoutS && opMode.opModeIsActive()) {


            //PID

            prevRunTime = time.seconds();

            average = Math.abs((getEncoderAverage() - initial)/ COUNTS_PER_INCH);
            error = distance - average;

            proportional = error * kP;

            integral += (error * (time.seconds() - prevRunTime)) * kI;


            derivative = ((error - lastError) / (time.seconds() - prevRunTime)) * kD;


            power = proportional + integral + derivative;





            if (power < .3 && kI == 0 && kD == 0) {
                power = .19;
            }

            //gyroHoloMovements


            angle_face = sensors.getGyroYawwwwwwwwwwwwwwwwwww() - face;


            motor_power = Holonomic.calcPowerAuto(heading, angle_face, rot_power);
            fl.setPower(motor_power[0] * power);
            fr.setPower(motor_power[1] * power);
            bl.setPower(motor_power[2] * power);
            br.setPower(motor_power[3] * power);

            if (angle_face >= 1) {
                rot_power = Math.abs(angle_face) / 20;
            } else if (angle_face <= -1) {
                rot_power = -Math.abs(angle_face) / 20;
            } else {
                rot_power = 0;
            }


            lastError = error;


        }


        choop();
    }

    public void gyroHoloPIDMovement(double heading, double face, double distance, double timeoutS) {
        ElapsedTime time = new ElapsedTime();
        ElapsedTime timeoutTimer = new ElapsedTime();

        double power;

        double kP = .7/24;
        double kI = .01;
        double kD = .001;

        double angle_face = 0;
        double rot_power = 0;

        double proportional;
        double integral = 0;
        double derivative;

        double[] motor_power = new double[4];

        double prevRunTime;

        double initial = getEncoderAverage();

        double average = Math.abs((getEncoderAverage() - initial)/ COUNTS_PER_INCH);

        double error = distance - average;
        double lastError = error;
        double powerFirst = 0;


        time.reset();
        timeoutTimer.reset();
        double control = .4;

        while (Math.abs(error) > 1 && timeoutTimer.seconds() < timeoutS && opMode.opModeIsActive()) {


            //PID

            prevRunTime = time.seconds();

            average = Math.abs((getEncoderAverage() - initial)/ COUNTS_PER_INCH);
            error = distance - average;



            proportional = error * kP;

            integral += (error * (time.seconds() - prevRunTime)) * kI;


            derivative = ((error - lastError) / (time.seconds() - prevRunTime)) * kD;


            power = (proportional + integral + derivative)*control;


            if(control < 1) {
                control += .01;
            }





            if (power < .3 && kI == 0 && kD == 0) {
                power = .19;
            }
            if (time.milliseconds() < 500)
            {
                powerFirst += .05;
                power = powerFirst;
            }

            //gyroHoloMovements


            angle_face = sensors.getGyroYawwwwwwwwwwwwwwwwwww()  - face;


            motor_power = Holonomic.calcPowerAuto(heading, angle_face, rot_power);
            fl.setPower(motor_power[0] * power);
            fr.setPower(motor_power[1] * power);
            bl.setPower(motor_power[2] * power);
            br.setPower(motor_power[3] * power);

            if (angle_face >= 1) {
                rot_power = Math.abs(angle_face) / 20;
            } else if (angle_face <= -1) {
                rot_power = -Math.abs(angle_face) / 20;
            } else {
                rot_power = 0;
            }


            lastError = error;


        }

        choop();
    }


}
