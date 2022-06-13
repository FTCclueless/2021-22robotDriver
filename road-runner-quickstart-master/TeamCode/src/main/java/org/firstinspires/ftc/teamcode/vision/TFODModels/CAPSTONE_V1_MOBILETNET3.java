package org.firstinspires.ftc.teamcode.vision.TFODModels;

import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

public class CAPSTONE_V1_MOBILETNET3 extends DetectionModel {
    private static final float MIN_CONFIDENCE = 0.55f;
    private static final float MIN_TRACKER_CONFIDENCE = 0.60f;
    private static final float MARGINAL_CORRELATION = 0.80f;
    private static final int INPUT_SIZE = 320;
    private static final boolean IS_QUANTIZED = false;

    public CAPSTONE_V1_MOBILETNET3(){
        super("MOBILENET_V3_LARGE_CAPSTONE_STICKER_100-10", "Capstone_v1_100-10.tflite",
                new String[] { "capstone" }, MIN_CONFIDENCE, INPUT_SIZE, IS_QUANTIZED);
    }

    @Override
    public TFObjectDetector.Parameters getParameters(int tfodMonitorViewId){
        TFObjectDetector.Parameters params = new TFObjectDetector.Parameters(tfodMonitorViewId);
        params.isModelQuantized = IS_QUANTIZED;
        params.maxFrameRate = 40;
        params.inputSize = INPUT_SIZE;
        params.minResultConfidence = MIN_CONFIDENCE;
        params.trackerMinCorrelation = MIN_TRACKER_CONFIDENCE;
        params.trackerMarginalCorrelation = MARGINAL_CORRELATION;
        params.numExecutorThreads = 1;
        return params;
    }
}
