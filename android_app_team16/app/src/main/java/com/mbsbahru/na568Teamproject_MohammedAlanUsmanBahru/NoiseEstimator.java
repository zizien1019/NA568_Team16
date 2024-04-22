package com.mbsbahru.na568Teamproject_MohammedAlanUsmanBahru;

import java.util.LinkedList;
import java.util.Queue;

public class NoiseEstimator {
    private Queue<float[]> accelDataQueue = new LinkedList<>();
    private Queue<float[]> gyroDataQueue = new LinkedList<>();
    private KalmanFilter camKF;
    private int windowSize = 100;  // number of samples to calculate variance
    public double[] accelVariances = new double[3];
    public double[] gyroVariances = new double[3];
    public NoiseEstimator(KalmanFilter camKF) {
        this.camKF = camKF;
    }

    public void addSample(float[] accelData, float[] gyroData) {
        if (accelDataQueue.size() >= windowSize) {
            accelDataQueue.poll();
            gyroDataQueue.poll();
        }
        accelDataQueue.add(accelData.clone());
        gyroDataQueue.add(gyroData.clone());

        if (accelDataQueue.size() == windowSize) {
            updateKalmanFilterNoiseParameters();
        }
    }

    private void updateKalmanFilterNoiseParameters() {
        accelVariances = calculateVariance(accelDataQueue);
        gyroVariances = calculateVariance(gyroDataQueue);

        camKF.setProcessNoiseStd(new double[] {Math.sqrt(gyroVariances[0]), Math.sqrt(gyroVariances[1]), Math.sqrt(gyroVariances[2]), 0.01, 0.01, 0.01});
        camKF.setMeasurementNoiseStd(new double[] {Math.sqrt(accelVariances[0]), Math.sqrt(accelVariances[1]), Math.sqrt(accelVariances[2]), 0.1, 0.1, 0.1});

//        camKF.setProcessNoiseStd(new double[] {Math.sqrt(gyroVariances[0]), Math.sqrt(gyroVariances[1]), Math.sqrt(gyroVariances[2]), 0.01});
//        camKF.setMeasurementNoiseStd(new double[] {Math.sqrt(accelVariances[0]), Math.sqrt(accelVariances[1]), Math.sqrt(accelVariances[2]), 0.1});

//        camKF.setProcessNoiseStd(new double[] {Math.sqrt(gyroVariances[0]), Math.sqrt(gyroVariances[1]), Math.sqrt(gyroVariances[2]), Math.sqrt(accelVariances[0]), Math.sqrt(accelVariances[1]), Math.sqrt(accelVariances[2])});
//        camKF.setMeasurementNoiseStd(new double[] {Math.sqrt(accelVariances[0]), Math.sqrt(accelVariances[1]), Math.sqrt(accelVariances[2]), Math.sqrt(gyroVariances[0]), Math.sqrt(gyroVariances[1]), Math.sqrt(gyroVariances[2])});
    }

    private double[] calculateVariance(Queue<float[]> dataQueue) {
        double[] mean = new double[3];
        double[] variance = new double[3];
        int n = dataQueue.size();

        for (float[] data : dataQueue) {
            mean[0] += data[0];
            mean[1] += data[1];
            mean[2] += data[2];
        }
        mean[0] /= n;
        mean[1] /= n;
        mean[2] /= n;

        for (float[] data : dataQueue) {
            variance[0] += Math.pow(data[0] - mean[0], 2);
            variance[1] += Math.pow(data[1] - mean[1], 2);
            variance[2] += Math.pow(data[2] - mean[2], 2);
        }
        variance[0] /= n;
        variance[1] /= n;
        variance[2] /= n;

        return variance;
    }
}
