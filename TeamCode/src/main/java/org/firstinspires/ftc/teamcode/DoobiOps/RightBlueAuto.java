/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.DoobiOps;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DoobiLibraries.Bezier;
import org.firstinspires.ftc.teamcode.DoobiLibraries.OdomClasses.OdomDriveTrain;
import org.firstinspires.ftc.teamcode.DoobiLibraries.Point;
import org.firstinspires.ftc.teamcode.DoobiLibraries.Wobble;

import java.util.ArrayList;

@Disabled
@Autonomous(name="RightBlueAuto", group="Linear Opmode")
public class RightBlueAuto extends LinearOpMode {

    // Declare OpMode members.
    ElapsedTime runtime;
    OdomDriveTrain drive;
    Wobble wobble;
    @Override
    public void runOpMode() {

        drive = new OdomDriveTrain(this);
        wobble = new Wobble(this);
        runtime = new ElapsedTime();

        ArrayList<Point> noRings0 = Bezier.interpolateSpline(Bezier.getVariables(0, 0, -7, 35, -14, 70));
        ArrayList<Point> noRings1 = Bezier.interpolateSpline(Bezier.getVariables(-10, -60, -13, -62, -16, -64));

        ArrayList<Point> OneRing0 = Bezier.interpolateSpline(Bezier.getVariables(0, 0, -10, -30, -10, -60));
        ArrayList<Point> OneRing1 = Bezier.interpolateSpline(Bezier.getVariables(-10, -60, 0, -70, 0, -80));
        ArrayList<Point> OneRing2 = Bezier.interpolateSpline(Bezier.getVariables(0, -80, 0, -72, 0, -64));

        ArrayList<Point> FourRings0 = Bezier.interpolateSpline(Bezier.getVariables(0, 0, -5, 30, -10, 60));
        ArrayList<Point> FourRings1 = Bezier.interpolateSpline(Bezier.getVariables(-10, 60, -13, 80, -16, 100));
        ArrayList<Point> FourRings2 = Bezier.interpolateSpline(Bezier.getVariables(-16, 100, -16, 82, -16, 64));

        waitForStart();
        //runtime.reset();
        switch (0) {
            case 0:
                //no rings
                drive.splineMove(noRings0, 1, 1, 5);
                //aim
                //shoot - wait for Nir to finish
                //sleep(1000);

                //drive.splineMove(noRings1, .5, 3);
                //wobble.wobbleDown();
                //wobble.hookUp();
                //wobble.wobbleUp();
                drive.end();
            case 1:
                //1 ring
                drive.splineMove(OneRing0, 1, 5, 5);
                //aim
                //shoot
                drive.splineMove(OneRing1, 1, 5, 5);
               // wobble.wobbleDown();
               // wobble.hookUp();
               // wobble.wobbleUp();
                drive.splineMove(OneRing2, 1, 5, 5);
                drive.end();

            case 4:
                drive.splineMove(FourRings0, 1, 5, 5);
                //aim
                //shoot
                drive.splineMove(FourRings1, .7, 3, 5);
                wobble.wobbleDown();
                wobble.hookUp();
                wobble.wobbleUp();
                drive.splineMove(FourRings2, .7, 3, 5);
                drive.end();
        }
    }
}
