package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CamHandler {

    private final HuskyLens huskyLens;
    private final boolean isRed;
    private final Telemetry telemetry;

    public CamHandler(HuskyLens lens, boolean isRed, Telemetry telemetry){
        this.huskyLens = lens;
        this.isRed = isRed;
        this.telemetry = telemetry;

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
    }

    /**
     * Adds telemetry for testing the camera
     */
    public void addLensTelemetry(){
        HuskyLens.Block[] blocks = huskyLens.blocks();
        for (int i = 0; i < blocks.length; i++) {
            telemetry.addData("Block " + i + " ID", blocks[i].id);
            telemetry.addData("Block " + i + " X", blocks[i].x);
            telemetry.addData("Block " + i + " Y", blocks[i].y);
        }
    }

    public double getTagError(int id){
        HuskyLens.Block[] tags = huskyLens.blocks(id);
        if (tags.length != 1) {
            return 0; // nothing found or multiple tags
        } else {
            HuskyLens.Block tag = tags[0];
            return 160 - tag.x; // (assuming 320 px width)
        }
    }
}
