package Opmodes.Auto.Pipelines;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.IntStream;

@Config
public class EdgeDetectionProcessPipeline extends OpenCvPipeline {
    public static double threshold1 = 10;
    public static double threshold2 = 70;
    public static int aperture = 3;
    public static int height = 10;
    public static int blurWidth = 5;
    public static int blurHeight = 3;
    public static int heightOffset = 0;
    public static int cameraOffset = 10;
    public float center;
    public boolean notFound;

    @Override
    public Mat processFrame(Mat input) {
        // Edge detection stuff, proably doesn't work very well.
        Rect rectCrop = new Rect(0, input.height() / 2 + heightOffset, input.width(), height);
        input = new Mat(input, rectCrop);

        Imgproc.blur(input, input, new Size(blurWidth, blurHeight));
        Imgproc.cvtColor(input, input, Imgproc.COLOR_BGR2GRAY);

        Mat out = new Mat();
        Imgproc.Canny(input, out, threshold1, threshold2, aperture);

        // Loop through the image and find lines.
        List<Integer> positions = new ArrayList<>();
        for (int i = 0; i < out.width(); i++) {
            for (int j = 0; j < out.height(); j++) {
                double[] pixelThing = out.get(j, i);
                double pixelVal = pixelThing[0];
                if (pixelVal > 0) {
                    positions.add(i);
                    break;
                }
            }
        }

        if (positions.size() == 0) {
            center = 0.5f;
            notFound = true;
            return out;
        }
        else {
            notFound = false;
        }

        IntStream thingy = positions.stream().mapToInt(Integer::intValue);
        float average = (float)thingy.sum() / (float)positions.size();

        center = average - input.width() / 2.0f + cameraOffset; // Offset so its relative to the center of the robot.

        return out;

//        Mat mat = new Mat();
//        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
//
//        Scalar blueLow = new Scalar(100, 50, 70);
//        Scalar blueHigh = new Scalar(128, 255, 255);
//        Mat thresh = new Mat();
//
//        Core.inRange(mat, blueLow, blueHigh, thresh);
//
//        Mat edges = new Mat();
//        Imgproc.Canny(thresh, edges, 100, 300);
//
//        // Take our edges and find the largest one
//        List<MatOfPoint> contours = new ArrayList<>();
//        Mat hierarchy = new Mat();
//        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
//
//        MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
//        Rect[] boundRect = new Rect[contours.size()];
//        for (int i = 0; i < contours.size(); i++) {
//            contoursPoly[i] = new MatOfPoint2f();
//            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
//            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
//        }
//
//        return edges;
    }
}
