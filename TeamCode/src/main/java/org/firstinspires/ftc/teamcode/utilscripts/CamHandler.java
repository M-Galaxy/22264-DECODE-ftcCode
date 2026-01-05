package org.firstinspires.ftc.teamcode.utilscripts;

import com.qualcomm.hardware.dfrobot.HuskyLens;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CamHandler {

    private final HuskyLens huskyLens;
    private final boolean isRed;
    private final Telemetry telemetry;

    private static final int FRAME_WIDTH = 320;

    public CamHandler(HuskyLens lens, boolean isRed, Telemetry telemetry){
        this.huskyLens = lens;
        this.isRed = isRed;
        this.telemetry = telemetry;

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
    }

    public void addLensTelemetry(){
        HuskyLens.Block[] blocks = huskyLens.blocks();

        if (blocks == null) {
            telemetry.addData("HuskyLens", "No blocks detected");
            return;
        }

        for (int i = 0; i < blocks.length; i++) {
            telemetry.addData("Block " + i + " ID", blocks[i].id);
            telemetry.addData("Block " + i + " X", blocks[i].x);
            telemetry.addData("Block " + i + " Y", blocks[i].y);
        }
    }

    public double getTagError(int id){
        HuskyLens.Block[] tags = huskyLens.blocks(id);

        if (tags == null || tags.length != 1) {
            return 0;
        }

        return (FRAME_WIDTH / 2.0) - tags[0].x;
    }

    public double getObolisk(){
        HuskyLens.Block[] tags = huskyLens.blocks();

        if (tags == null || tags.length != 1) {
            return 0;
        }
        return tags[0].id;
    }
}
