package org.firstinspires.ftc.teamcode.vision.Tensorflow;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.vision.TFODModels.DetectionModel;

import java.util.ArrayList;
import java.util.List;

public class ObjectDetector {
    public enum MatchMode {
        DEFAULT, SIMPLE_MEAN, WEIGHTED_MEAN, LARGEST_AREA, SMALLEST_AREA
    }
    private static final String VUFORIA_KEY =
            "ARjSEzX/////AAABmTyfc/uSOUjluYpQyDMk15tX0Mf3zESzZKo6V7Y0O/qtPvPQOVben+DaABjfl4m5YNOhGW1HuHywuYGMHpJ5/uXY6L8Mu93OdlOYwwVzeYBhHZx9le+rUMr7NtQO/zWEHajiZ6Jmx7K+A+UmRZMpCmr//dMQdlcuyHmPagFERkl4fdP0UKsRxANaHpwfQcY3npBkmgE8XsmK4zuFEmzfN2/FV0Cns/tiTfXtx1WaFD0YWYfkTHRyNwhmuBxY6MXNmaG8VlLwJcoanBFmor2PVBaRYZ9pnJ4TJU5w25h1lAFAFPbLTz1RT/UB3sHT5CeG0bMyM4mTYLi9SHPOUQjmIomxp9D7R39j8g5G7hiKr2JP";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private final HardwareMap hm;
    private DetectionModel model;
    private final String webcam;

    private List<Recognition> updatedRecognitions;

    private double[] zoomRange = new double[] { 1.0, 2.2 };
    private double zoomChangeRate = 0.025;
    private long prevZoomTime;
    private long zoomChangeDelay = 50L;

    private double currZoom = 1.0;

    public ObjectDetector(DetectionModel tfodModel, HardwareMap hm, String webcamDeviceName){
        this.hm = hm;
        model = tfodModel;
        webcam = webcamDeviceName;

        updatedRecognitions = new ArrayList<>();
    }

    public ObjectDetector(DetectionModel tfodModel, HardwareMap hm){
        this.hm = hm;
        model = tfodModel;
        webcam = null;

        updatedRecognitions = new ArrayList<>();
    }

    public void initialize(){
        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.0, 16.0/9.0);
            prevZoomTime = System.currentTimeMillis();
        }
    }

    public void autoAdjustZoom(){
        if ((updatedRecognitions == null || updatedRecognitions.isEmpty()) && System.currentTimeMillis() - prevZoomTime > zoomChangeDelay) {
            currZoom += zoomChangeRate;
            prevZoomTime = System.currentTimeMillis();
            if ((currZoom + zoomChangeRate > zoomRange[1] && zoomChangeRate > 0) || (currZoom + zoomChangeRate < zoomRange[0] && zoomChangeRate < 0))
                zoomChangeRate = -zoomChangeRate;
            setZoom(currZoom, 16.0 / 9.0);
        } else if(updatedRecognitions != null && !updatedRecognitions.isEmpty() && zoomChangeRate > 0){
            zoomChangeRate = -zoomChangeRate;
        }
    }

    public void setZoom(double magnification, double aspectRatio){
        if (tfod != null) {
            if(magnification < 1.0){
                magnification = 1.0;
            }
            tfod.setZoom(magnification, aspectRatio);
        }
    }

    public void setZoom(double magnification){
        if (tfod != null) {
            tfod.setZoom(magnification, 16.0/9.0);
        }
    }

    public List<Recognition> getDetections() {
        if(tfod != null){
            if(updatedRecognitions == null) {
                updatedRecognitions = new ArrayList<>();
            }

            List<Recognition> recognitions = tfod.getUpdatedRecognitions();

            if(recognitions != null) {
                updatedRecognitions.clear();
                updatedRecognitions.addAll(recognitions);
            }
        }
        return updatedRecognitions;
    }

    public Double getPositionX(MatchMode mode){
        Double objectX = null;
        if(tfod != null){
            List<Recognition> updatedRecognitions = getDetections();
            if (updatedRecognitions == null || updatedRecognitions.isEmpty()) return null;

            switch (mode){
                case DEFAULT:
                    double sum = 0.0;
                    for (Recognition r : updatedRecognitions)
                        sum += (r.getLeft() + r.getRight()) / 2.0;

                    objectX = sum / updatedRecognitions.size();
                    break;
                case SIMPLE_MEAN:
                    sum = 0.0;
                    for (Recognition r : updatedRecognitions)
                        sum += (r.getLeft() + r.getRight()) / 2.0;

                    objectX = sum / updatedRecognitions.size();
                    break;
                case WEIGHTED_MEAN:
                    double bias = 0.5; // [ 0.0, 1.0 ], Larger = higher bias towards small-area boxes

                    double largestArea = 0.0;
                    for (Recognition r : updatedRecognitions) {
                        if (largestArea < r.getHeight() * r.getWidth()) {
                            largestArea = r.getHeight() * r.getWidth();
                        }
                    }

                    double weightedSum = 0.0;
                    double sumWeights = 0.0;
                    for (Recognition r : updatedRecognitions) {
                        double weight = (1 - (r.getHeight() * r.getWidth() / largestArea)) * bias + (1 - bias);
                        System.out.println(weight);
                        System.out.println(r.getHeight() * r.getWidth());
                        System.out.println(largestArea);
                        System.out.println(r.getHeight() * r.getWidth() / largestArea);
                        System.out.println(1 - (r.getHeight() * r.getWidth() / largestArea));
                        sumWeights += weight;
                        weightedSum += (r.getLeft() + r.getRight()) / 2.0 * weight;
                    }

                    objectX = weightedSum / sumWeights;
                    break;
                case SMALLEST_AREA:
                    double minArea = Double.MAX_VALUE;
                    for (Recognition r : updatedRecognitions) {
                        if (minArea > r.getHeight() * r.getWidth()) {
                            minArea = r.getHeight() * r.getWidth();
                            objectX = (r.getLeft() + r.getRight()) / 2.0;
                        }
                    }
                    break;
                case LARGEST_AREA:
                    double maxArea = 0.0;
                    for (Recognition r : updatedRecognitions) {
                        if (maxArea < r.getHeight() * r.getWidth()) {
                            maxArea = r.getHeight() * r.getWidth();
                            objectX = (r.getLeft() + r.getRight()) / 2.0;
                        }
                    }
                    break;
            }
        }
        return objectX;
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        if(webcam != null) {
            parameters.cameraName = hm.get(WebcamName.class, webcam);
        } else {
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        }

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hm.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hm.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = model.getParameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(model.MODEL_ASSET_FILE_PATH, model.ELEMENT_NAMES);
    }
}
