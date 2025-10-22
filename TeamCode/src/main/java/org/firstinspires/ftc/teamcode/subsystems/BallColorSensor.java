package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class BallColorSensor {
    public enum DetectedColor {
        PURPLE,
        GREEN,
        UNKNOWN
    }

    private String sensorName = "colorSensor";
    private NormalizedColorSensor colorSensor;
    private boolean ledAvailable;

    private final float[] hsv = new float[3];
    private float lastHue = Float.NaN;
    private float lastSaturation = 0f;
    private float lastValue = 0f;
    private DetectedColor lastDetected = DetectedColor.UNKNOWN;

    private float purpleHueMin = 260f;
    private float purpleHueMax = 320f;
    private float greenHueMin = 80f;
    private float greenHueMax = 160f;
    private float minSaturation = 0.15f;
    private float minValue = 0.05f;

    public void setSensorName(String sensorName) {
        this.sensorName = sensorName;
    }

    public void init(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, sensorName);
        if (colorSensor instanceof SwitchableLight) {
            SwitchableLight light = (SwitchableLight) colorSensor;
            light.enableLight(true);
            ledAvailable = true;
        } else ledAvailable = false;
    }

    public boolean isInitialized() {
        return colorSensor != null;
    }

    public DetectedColor update() {
        if (colorSensor == null) {
            lastDetected = DetectedColor.UNKNOWN;
            return lastDetected;
        }

        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsv);
        lastHue = hsv[0];
        lastSaturation = hsv[1];
        lastValue = hsv[2];

        lastDetected = classifyCurrentReading();
        return lastDetected;
    }

    private DetectedColor classifyCurrentReading() {
        if (Float.isNaN(lastHue) || lastSaturation < minSaturation || lastValue < minValue) return DetectedColor.UNKNOWN;

        if (hueInRange(lastHue, purpleHueMin, purpleHueMax)) return DetectedColor.PURPLE;

        if (hueInRange(lastHue, greenHueMin, greenHueMax)) return DetectedColor.GREEN;

        return DetectedColor.UNKNOWN;
    }

    private boolean hueInRange(float hue, float min, float max) {
        hue = normalizeHue(hue);
        min = normalizeHue(min);
        max = normalizeHue(max);

        if (min <= max) return hue >= min && hue <= max;

        return hue >= min || hue <= max;
    }

    private float normalizeHue(float hue) {
        float result = hue % 360f;
        if (result < 0f) result += 360f;
        return result;
    }

    public DetectedColor getLastDetectedColor() {
        return lastDetected;
    }

    public float getLastHue() {
        return lastHue;
    }

    public float getLastSaturation() {
        return lastSaturation;
    }

    public float getLastValue() {
        return lastValue;
    }

    public void setPurpleHueRange(float minHue, float maxHue) {
        purpleHueMin = minHue;
        purpleHueMax = maxHue;
    }

    public void setGreenHueRange(float minHue, float maxHue) {
        greenHueMin = minHue;
        greenHueMax = maxHue;
    }

    public void setMinimumSaturation(float minSaturation) {
        this.minSaturation = minSaturation;
    }

    public void setMinimumValue(float minValue) {
        this.minValue = minValue;
    }

    public boolean isLedAvailable() {
        return ledAvailable;
    }

    public void setLedEnabled(boolean enabled) {
        if (colorSensor instanceof SwitchableLight) ((SwitchableLight) colorSensor).enableLight(enabled);
    }
}
