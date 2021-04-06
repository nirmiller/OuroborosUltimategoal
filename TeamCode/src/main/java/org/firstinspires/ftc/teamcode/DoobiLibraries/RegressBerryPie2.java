package org.firstinspires.ftc.teamcode.DoobiLibraries;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class RegressBerryPie2 {
    private static final String asset = "UltimateGoal.tflite";
    //TODO: if 1 and 4 are switched, go to the labels.txt file in assets and switch where Single and Quad is
    private static final String quad = "Quad";
    private static final String single = "Single";

    private static final String key =
            "AQdAZAn/////AAABmTMBJaMcIUWAiBTGUzGG95SDuyikdfzZayCTxqFCKhHnJqiaqXa7qw0UZsekRNRaL" +
                    "L7mV8EnwA+AcHfZSTIcMdmxwAdbgb2u1NHpFGUUi/nkG3fLMeRfqbJrUrJmIcOx9CK21XaK6u" +
                    "uU7MAQ4b1+YbGcAbTRwn0Pj1I8t5/lAK+QZCZZIrOq7FLEr4WLEHWVJVmB8xhky5ZJivcwEkyOk" +
                    "+BCoSSAkgkWRD8SknX3TJRx43IbWw4eesgYgmBkQ3YEZ4CynWssTihgLf4PFjp2vGpzZEcacly8" +
                    "72OQqkynF/PRG0GB2RrDcQkp1LRH0tTcPTAQBBw7v+3zH7Ou03vbxBk+njhsPUj22l34VKmM8Jod";

    private VuforiaLocalizer regress;
    private TFObjectDetector berry;

    LinearOpMode opMode;
    public RegressBerryPie2(LinearOpMode opMode) {

        this.opMode = opMode;
        initVuforia();
        initTfod();
    }

    public int pie() {
        //initVuforia();
        //initTfod();

        if (berry != null) {
            berry.activate();
            //TODO: May need to adjust zoom
            berry.setZoom(1, 1.78);
        }

        ElapsedTime time = new ElapsedTime();
        int stackSize = 0;

        if (opMode.opModeIsActive()) {
            while (opMode.opModeIsActive() && time.milliseconds() <= 1000) {
                if (berry != null) {
                    List<Recognition> updatedRecognitions = berry.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(quad)) stackSize = 4;
                            else stackSize = 1;
                        }
                        opMode.telemetry.update();
                    }
                }
            }
        }

        if (berry != null) berry.shutdown();
        return stackSize;
    }

    private void initVuforia() {
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id",
                        opMode.hardwareMap.appContext.getPackageName());
        opMode.telemetry.addLine("here 1");
        opMode.telemetry.update();

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = key;
        opMode.telemetry.addLine("here 2");
        opMode.telemetry.update();

        parameters.cameraName = opMode.hardwareMap.get(WebcamName.class, "Webcam 1");
        opMode.telemetry.addLine("here 3");
        opMode.telemetry.update();

        regress = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        opMode.telemetry.addLine("here 4");
        opMode.telemetry.update();
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        berry = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, regress);
        berry.loadModelFromAsset(asset, single, quad);
    }
}
