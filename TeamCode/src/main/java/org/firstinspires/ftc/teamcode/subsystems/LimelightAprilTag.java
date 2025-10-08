package org.firstinspires.ftc.teamcode;

import org.json.JSONArray;
import org.json.JSONObject;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.URL;

/*
 * Limelight AprilTag detection system
 * -Connect Limelight to robot network
 * -Fetch AprilTag pose for a specific tag ID
 */
public class LimelightAprilTag {
    private final String limelightHost; //limelight network address
    private final int targetTagId; //the id we need
    public LimelightAprilTag(String limelightHost, int targetTagId) {
        this.limelightHost = limelightHost;
        this.targetTagId = targetTagId;
    }
    //this just gets the translation vector (t) from the limelight json data
    public double[] getTargetPose() {
        try {
            URL url = new URL(limelightHost+"/json"); //fetch by https
            HttpURLConnection conn = (HttpURLConnection) url.openConnection();
            conn.setRequestMethod("GET");
            BufferedReader in = new BufferedReader(new InputStreamReader(conn.getInputStream()));
            StringBuilder response = new StringBuilder();
            String line;
            while ((line=in.readLine())!=null) response.append(line);
            in.close();
            JSONObject json = new JSONObject(response.toString());
            if (!json.has("Results")) return null;
            JSONObject results = json.getJSONObject("Results");
            JSONArray targets = results.getJSONArray("FiducialResults");
            if (targets.length()==0) return null;
            for (int i=0; i<targets.length(); i++) {
                JSONObject tag = targets.getJSONObject(i);
                int id = tag.getInt("id"); //tag id
                if (id==targetTagId) {
                    JSONArray tvec = tag.getJSONArray("t"); //<x,y,z> in meters
                    double x = tvec.getDouble(0); double y = tvec.getDouble(1); double z = tvec.getDouble(2);
                    return new double[]{x, y, z};
                }
            }
            return null; //no tag seen
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }
    //compute distance from camera to tag in meters (ignores y cuz its irrelevant for the shooter calculation)
    public double getDistanceMeters() {
        double[] pose = getTargetPose();
        if (pose==null) return -1;
        double x = pose[0]; double z = pose[2];
        return Math.sqrt(x*x+z*z);
    }
}
