package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

//@Config
@TeleOp(name="⚙ Color Sensor Tuner (Improved)", group="⚙ Tuning")
public class colorTuning extends LinearOpMode {

    public NormalizedColorSensor colorSensor;

    // HSV ranges (tweak if needed)
    public static float PURPLE_HUE_LOW = 250;
    public static float PURPLE_HUE_HIGH = 310; // wrap-around handled
    public static float GREEN_HUE_LOW = 70;
    public static float GREEN_HUE_HIGH = 160;
    public static float ORANGE_HUE_LOW = 10;
    public static float ORANGE_HUE_HIGH = 50;

    public static float MIN_BRIGHTNESS = 0.002f; // alpha threshold for object detection

    @Override
    public void runOpMode() throws InterruptedException {

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSens");
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Color Sensor Tuner Ready");
        telemetry.addLine("Open FTC Dashboard to tune color targets.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            float r = colors.red;
            float g = colors.green;
            float b = colors.blue;
            float a = colors.alpha;

            // ========== OBJECT DETECTION ==========
            boolean objectPresent = a > MIN_BRIGHTNESS;

            // ========== NORMALIZED RGB RATIOS ==========
            float total = r + g + b;
            float rn = total > 0 ? r / total : 0;
            float gn = total > 0 ? g / total : 0;
            float bn = total > 0 ? b / total : 0;

            // ========== HSV ==========
            float[] hsv = new float[3];
            android.graphics.Color.RGBToHSV(
                    (int)(r * 255),
                    (int)(g * 255),
                    (int)(b * 255),
                    hsv
            );
            float hue = hsv[0];

            // ========== COLOR DETECTION ==========
            boolean isPurple = false, isGreen = false, isOrange = false;

            if(objectPresent){
                // Purple: handle wrap-around hue
                isPurple = (hue >= PURPLE_HUE_LOW || hue <= PURPLE_HUE_HIGH) && bn > rn && bn > gn;
                isGreen  = (hue >= GREEN_HUE_LOW && hue <= GREEN_HUE_HIGH) && gn > rn && gn > bn;
                isOrange = (hue >= ORANGE_HUE_LOW && hue <= ORANGE_HUE_HIGH) && rn > gn && rn > bn;
            }

            // ========== DRIVER HUB TELEMETRY ==========
            telemetry.addLine("=== RAW COLOR ===");
            telemetry.addData("R", r);
            telemetry.addData("G", g);
            telemetry.addData("B", b);
            telemetry.addData("A", a);

            telemetry.addLine("\n=== HSV ===");
            telemetry.addData("Hue", hue);
            telemetry.addData("Sat", hsv[1]);
            telemetry.addData("Val", hsv[2]);

            telemetry.addLine("\n=== DETECTION ===");
            telemetry.addData("Object Present?", objectPresent);
            telemetry.addData("Purple?", isPurple);
            telemetry.addData("Green?", isGreen);
            telemetry.addData("Orange?", isOrange);

            telemetry.update();

            // ========== DASHBOARD TELEMETRY ==========
//            TelemetryPacket packet = new TelemetryPacket();
//            packet.put("R", r);
//            packet.put("G", g);
//            packet.put("B", b);
//            packet.put("Hue", hue);
//            packet.put("Sat", hsv[1]);
//            packet.put("Val", hsv[2]);
//
//            packet.put("Object Present?", objectPresent);
//            packet.put("Purple?", isPurple);
//            packet.put("Green?", isGreen);
//            packet.put("Orange?", isOrange);
//
//            packet.fieldOverlay()
//                    .setFill("rgb(" + (int)(r*255) + "," + (int)(g*255) + "," + (int)(b*255) + ")")
//                    .fillRect(10, 10, 50, 50);
//
//            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}
