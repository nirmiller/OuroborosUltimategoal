package org.firstinspires.ftc.teamcode.DoobiLibraries;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

public class RegressBerryPie {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String key =
            "AQdAZAn/////AAABmTMBJaMcIUWAiBTGUzGG95SDuyikdfzZayCTxqFCKhHnJqiaqXa7qw0UZsekRNRaL" +
                    "L7mV8EnwA+AcHfZSTIcMdmxwAdbgb2u1NHpFGUUi/nkG3fLMeRfqbJrUrJmIcOx9CK21XaK6u" +
                    "uU7MAQ4b1+YbGcAbTRwn0Pj1I8t5/lAK+QZCZZIrOq7FLEr4WLEHWVJVmB8xhky5ZJivcwEkyOk" +
                    "+BCoSSAkgkWRD8SknX3TJRx43IbWw4eesgYgmBkQ3YEZ4CynWssTihgLf4PFjp2vGpzZEcacly8" +
                    "72OQqkynF/PRG0GB2RrDcQkp1LRH0tTcPTAQBBw7v+3zH7Ou03vbxBk+njhsPUj22l34VKmM8Jod";

    private VuforiaLocalizer regress;
    private TFObjectDetector berry;

    LinearOpMode opMode;
    public RegressBerryPie(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public int pie() {
        initVuforia();
        initTfod();

        if (berry != null) {
            berry.activate();
            berry.setZoom(2.5, 1.78);
        }

        ElapsedTime time = new ElapsedTime();
        int stackSize = 0;

        if (opMode.opModeIsActive()) {
            while (opMode.opModeIsActive() && time.milliseconds() >= 500) {
                if (berry != null) {
                    List<Recognition> updatedRecognitions = berry.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        for (Recognition recognition : updatedRecognitions) {
                          if (recognition.getLabel().equals(LABEL_FIRST_ELEMENT)) stackSize = 4;
                          else stackSize = 1;
                        }
                        opMode.telemetry.update();
                    }
                    else stackSize = 0;
                }
            }
        }

        if (berry != null) berry.shutdown();
        return stackSize;
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = key;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        regress = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
       tfodParameters.minResultConfidence = 0.8f;
       berry = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, regress);
       berry.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
