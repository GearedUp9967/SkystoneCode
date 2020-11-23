package org.firstinspires.ftc.teamcode;


import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

public class StoneVision extends OpenCvPipeline {
    public int width;
    public int height;
    private Mat hsv = new Mat();
    private Mat yellow = new Mat();
    private Mat display = new Mat();
    private Mat left = new Mat();
    private Mat middle = new Mat();
    private Mat right = new Mat();
    private Scalar red = new Scalar(255,0,0);
    private Scalar blue = new Scalar(0,0,255);
    private Scalar purple = new Scalar(255,0,255);
    private Scalar green = new Scalar(0,255,0);
    private int num=0;
    private int line;
    private int blockwid;


    public int numPixels=0;


    private Position pos= Position.UNKNOWN;

    private boolean redSide=false;

    public enum Position {
        LEFT,
        MIDDLE,
        RIGHT,
        UNKNOWN
    }

    public StoneVision(boolean redside) {
        redSide=redside;
    }

    @Override
    public Mat processFrame(Mat rgba) {
        width = rgba.width();
        height = rgba.height();
        Imgproc.cvtColor(rgba, hsv, Imgproc.COLOR_RGB2HSV, 3);

        Core.inRange(hsv, new Scalar(10, 100, 100), new Scalar(30, 255, 255), yellow);
        double top=235/480.0 * height;
        double bottom=330/480.0 * height;
        double leftcoord=29.0/480.0 * width;
        double rightcoord=width-29.0/480.0 * width;
        double boxwidth=(int)((rightcoord-leftcoord)/3.0);
        double boxheight=(int)(bottom-top);
        Point topleft=new Point(leftcoord,top);
        Point bottomright=new Point(rightcoord,bottom);

        blockwid=(int) (width*632.0/2448.0);

        //dumbly changing the line height by using block width as a reference from the display
        line=(int) ((height*2200.0/3264.0)+15);




        Rect leftRect=new Rect((int) leftcoord,(int) top,(int) boxwidth,(int) boxheight);
        Rect middleRect=new Rect((int) (leftcoord+boxwidth),(int) top,(int) boxwidth,(int) boxheight);
        Rect rightRect=new Rect((int) (leftcoord+2*boxwidth),(int) top,(int) boxwidth,(int) boxheight);

        left=yellow.submat(leftRect);
        middle=yellow.submat(middleRect);
        right=yellow.submat(rightRect);


        Imgproc.cvtColor(yellow,display,Imgproc.COLOR_GRAY2RGB,3);



        if(redSide)
        {
            numPixels=scanPixels(false);
            pos=getPosFromPixels(numPixels,true);
        }
        else
        {
            numPixels=scanPixels(true);
            pos=getPosFromPixels(numPixels,false);
        }


        for(int i=0;i<width;i++) {
            if(Core.countNonZero(yellow.submat(new Rect(i,line,1,1)))==0) {
                Imgproc.rectangle(display, new Point(i, line), new Point(i, line), red, 2);
            }
            else
            {
                Imgproc.rectangle(display, new Point(i, line), new Point(i, line), green, 2);
            }

        }


        return display;
    }


    private int scanPixels(boolean rightToLeft)
    {

        int start=0;
        int inc=1;
        if(!rightToLeft)
        {
            start=width-1;
            inc=-1;
        }


        boolean startedCounting=false;
        int blackGap=0;
        int pixels=0;
        int i=start;
        Position position=Position.UNKNOWN;
        while(0<=i && i<width) {
            if(Core.countNonZero(yellow.submat(new Rect(i,line,1,1)))==0) {
                if(startedCounting)
                {
                    blackGap+=1;
                    if(blackGap>10)
                    {
                        break;
                    }
                }
            }
            else
            {
                blackGap=0;
                startedCounting=true;
                pixels+=1;
            }

            i+=inc;
        }


        return pixels;
    }

    private Position getPosFromPixels(int pixels,boolean redSide) {
        if(!redSide) {

            if(pixels<30) {
                return Position.LEFT;
            }

            else if(Math.abs(pixels-blockwid)<Math.abs(pixels-2*blockwid))
            {
                return Position.MIDDLE;
            }
            else
            {
                return Position.RIGHT;
            }
        }
        else {
            if(pixels<30) {
                return Position.RIGHT;
            }

            else if(Math.abs(pixels-blockwid)<Math.abs(pixels-2*blockwid))
            {
                return Position.MIDDLE;
            }
            else
            {
                return Position.LEFT;
            }
        }
    }

    public Position getPos() {
        return pos;
    }



}
