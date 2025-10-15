package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

import java.util.Collections;
import java.util.Comparator;
import java.util.Iterator;
import java.util.List;

/**
 * Small collection of utility functions to filter and choose blobs.
 */
public class BlobUtils {

    public static void filterByContourArea(List<ColorBlobLocatorProcessor.Blob> blobs, double minArea, double maxArea) {
        if (blobs == null) return;
        Iterator<ColorBlobLocatorProcessor.Blob> it = blobs.iterator();
        while (it.hasNext()) {
            ColorBlobLocatorProcessor.Blob b = it.next();
            double area = b.getContourArea();
            if (area < minArea || area > maxArea) it.remove();
        }
    }

    public static void filterByCircularity(List<ColorBlobLocatorProcessor.Blob> blobs, double minCircularity, double maxCircularity) {
        if (blobs == null) return;
        Iterator<ColorBlobLocatorProcessor.Blob> it = blobs.iterator();
        while (it.hasNext()) {
            ColorBlobLocatorProcessor.Blob b = it.next();
            double c = b.getCircularity();
            if (Double.isNaN(c) || c < minCircularity || c > maxCircularity) it.remove();
        }
    }

    /** Sorts descending by contour area */
    public static void sortByAreaDesc(List<ColorBlobLocatorProcessor.Blob> blobs) {
        Collections.sort(blobs, new Comparator<ColorBlobLocatorProcessor.Blob>() {
            @Override
            public int compare(ColorBlobLocatorProcessor.Blob o1, ColorBlobLocatorProcessor.Blob o2) {
                return Double.compare(o2.getContourArea(), o1.getContourArea());
            }
        });
    }

    /** Returns the largest blob (or null) */
    public static ColorBlobLocatorProcessor.Blob largestBlob(List<ColorBlobLocatorProcessor.Blob> blobs) {
        if (blobs == null || blobs.isEmpty()) return null;
        sortByAreaDesc(blobs);
        return blobs.get(0);
    }

    /** Safe accessor for circle fit */
    public static Circle getCircle(ColorBlobLocatorProcessor.Blob b) {
        if (b == null) return null;
        try {
            return b.getCircle();
        } catch (Exception e) {
            return null;
        }
    }
}
