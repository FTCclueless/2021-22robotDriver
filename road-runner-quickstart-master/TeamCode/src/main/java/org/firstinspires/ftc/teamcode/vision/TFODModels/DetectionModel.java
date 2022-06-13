package org.firstinspires.ftc.teamcode.vision.TFODModels;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.Arrays;

public abstract class DetectionModel {
    public final String MODEL_NAME;
    public final String MODEL_ASSET_FILE_PATH;
    public final String[] ELEMENT_NAMES;

    public float MIN_CONFIDENCE;
    public int INPUT_SIZE;
    public boolean IS_QUANTIZED;

    protected DetectionModel(String modelName, String modelAssetFile, String[] elements,
                              float minConfidence, int inputSize, boolean isQuantized){
        MODEL_NAME = modelName;
        MODEL_ASSET_FILE_PATH = modelAssetFile;
        ELEMENT_NAMES = elements;
        MIN_CONFIDENCE = minConfidence;
        INPUT_SIZE = inputSize;
        IS_QUANTIZED = isQuantized;
    }

    public TFObjectDetector.Parameters getParameters(int tfodMonitorViewId){
        return null;
    }

    @Override
    @NonNull
    public String toString(){
        return "{" + MODEL_NAME + ": " + MODEL_ASSET_FILE_PATH + " - " + Arrays.toString(ELEMENT_NAMES) + "}";
    }
}
