#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "cv-helpers.hpp"
// #include <vector.h>
#include <vector>
#include <iostream>
#include <fstream>
int main(int argc, char * argv[]) try
{
    std::ofstream outFile;
    outFile.open("/home/brl/realsense/data/1.csv");

    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer  color_map;
    rs2::align      align_to(RS2_STREAM_COLOR);
    rs2::pipeline   pipe;
    // Start streaming with default recommended configuration
    pipe.start();

    const auto window1 = "Display Image1";
    const auto window2 = "Display Image2";
    const auto window3 = "Display Image3";
    cv::namedWindow(window1, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(window2, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(window3, cv::WINDOW_AUTOSIZE);

    while (cv::waitKey(1) < 0 && cv::getWindowProperty(window1, cv::WND_PROP_AUTOSIZE) >= 0)
    {
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        rs2::frameset aligned_set = align_to.process(data);
        rs2::frame depth = data.get_depth_frame().apply_filter(color_map);
        rs2::frame color = data.get_color_frame().apply_filter(color_map);
        auto color_mat = frame_to_mat(aligned_set.get_color_frame());

        // Query frame size (width and height)
        // const int w = depth.as<rs2::video_frame>().get_width();
        // const int h = depth.as<rs2::video_frame>().get_height();
        const int w = color.as<rs2::video_frame>().get_width();
        const int h = color.as<rs2::video_frame>().get_height();
        // Create OpenCV matrix of size (w,h) from the colorized depth data
        cv::Mat image(cv::Size(w, h), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);

        cv::Mat image_;
        cv::cvtColor(image, image_, cv::COLOR_BGR2RGB);


        int iLowH   = 120;
        int iHighH  = 150;
        int iLowS   = 50;
        int iHighS  = 255;
        int iLowV   = 50;
        int iHighV  = 255;

        //Create trackbars in "Control" window
        cv::createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
        cv::createTrackbar("HighH", "Control", &iHighH, 179);

        cv::createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
        cv::createTrackbar("HighS", "Control", &iHighS, 255);

        cv::createTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
        cv::createTrackbar("HighV", "Control", &iHighV, 255);

        int iLastX = -1; 
        int iLastY = -1;
        //Create a black image with the size as the camera output
        cv::Mat imgLines = cv::Mat::zeros(image.size(), CV_8UC3);

        cv::Mat imageHSV;
        cv::cvtColor(image, imageHSV, cv::COLOR_BGR2HSV);
        cv::Mat imgThresholded;

        cv::inRange(imageHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

        //morphological opening (remove small objects from the foreground)
        cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
        cv::dilate(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 

        //morphological closing (fill small holes in the foreground)
        cv::dilate(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 
        cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

        //Calculate the moments of the thresholded image
        cv::Moments oMoments = cv::moments(imgThresholded);

        cv::Mat canny_output;
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        Canny(imgThresholded, canny_output, 50, 150, 3 );

        // find contours
        cv::findContours(canny_output, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

        std::vector<cv::Moments> mu(contours.size());
        for( int i = 0; i<contours.size(); i++ )
        { mu[i] = cv::moments( contours[i], false ); }
 
        // get the centroid of figures.
        std::vector<cv::Point2f> mc(contours.size());
        for( int i = 0; i<contours.size(); i++)
        { mc[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); }
 
        // draw contours
        cv::Mat drawing(canny_output.size(), CV_8UC3, cv::Scalar(255,255,255));
        // std::cout<<contours.size()<<std::endl;
        if(contours.size()== 20)
        {
            for( int i = 0; i<contours.size(); i++ )
            {
                cv::Scalar color = cv::Scalar(167,151,0); // B G R values
                cv::drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point());
                cv::circle( drawing, mc[i], 4, color, -1, 8, 0 );
                outFile <<mc[i].x<<","<<mc[i].y<<",";
                // std::cout<<mc[i].x<<","<<mc[i].y<<" ";
            }
            outFile<<"\n";
        }
        double dM01 = oMoments.m01;
        double dM10 = oMoments.m10;
        double dArea = oMoments.m00;

        // if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
        if (dArea > 10000)
        {
            //calculate the position of the ball
            int posX = dM10 / dArea;
            int posY = dM01 / dArea;        
                
            if (iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0)
            {
                //Draw a red line from the previous point to the current point
                cv::line(imgLines, cv::Point(posX, posY), cv::Point(iLastX, iLastY), cv::Scalar(0,0,255), 2);
            }

            iLastX = posX;
            iLastY = posY;
        }
        // Update the window with new data
        cv::imshow(window1, image_);
        cv::imshow(window2, imgThresholded);
        cv::imshow(window3, drawing);


    }
    outFile.close();
    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
