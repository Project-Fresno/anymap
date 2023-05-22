#ifndef LANE_EXTENSION_H_
#define LANE_EXTENSION_H_

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <cmath>



std::vector<std::vector<cv::Point>> find_contours(cv::Mat frame) {

  // std::cout << "finding contours\n";
  cv::Mat grayscale_image = frame;

  // cv::cvtColor(frame, grayscale_image, cv::COLOR_BGR2GRAY);
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(grayscale_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  // std::cout << "found contours\n";
  return contours;
}


std::string type2str(int type) {
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}


namespace lane_extension {
    std::vector<cv::Point2f> order_points_new(const std::vector<cv::Point2f>& pts) {
        // Sort the points based on their x-coordinates
        std::vector<cv::Point2f> xSorted = pts;
        std::sort(xSorted.begin(), xSorted.end(), [](cv::Point2f a, cv::Point2f b) {
            return a.x < b.x;
        });

        // Grab the left-most and right-most points from the sorted
        // x-coordinate points
        std::vector<cv::Point2f> leftMost = {xSorted[0], xSorted[1]};
        std::vector<cv::Point2f> rightMost = {xSorted[2], xSorted[3]};

        // Now, sort the left-most coordinates according to their
        // y-coordinates so we can grab the top-left and bottom-left
        // points, respectively
        std::sort(leftMost.begin(), leftMost.end(), [](cv::Point2f a, cv::Point2f b) {
            return a.y < b.y;
        });
        cv::Point2f tl = leftMost[0];
        cv::Point2f bl = leftMost[1];

        // Now, sort the right-most coordinates according to their
        // y-coordinates so we can grab the top-right and bottom-right
        // points, respectively
        std::sort(rightMost.begin(), rightMost.end(), [](cv::Point2f a, cv::Point2f b) {
            return a.y < b.y;
        });
        cv::Point2f tr = rightMost[0];
        cv::Point2f br = rightMost[1];

        // Return the coordinates in top-left, top-right,
        // bottom-right, and bottom-left order
        return {tl, tr, br, bl};
    }


    std::vector<cv::Point2f> order_points_x(std::vector<cv::Point2f> pts) {
        // sort the points based on their x-coordinates
        std::sort(pts.begin(), pts.end(), [](cv::Point2f a, cv::Point2f b) { return a.x < b.x; });

        // grab the left-most and right-most points from the sorted
        // x-roodinate points
        std::vector<cv::Point2f> leftMost(pts.begin(), pts.begin() + 2);
        std::vector<cv::Point2f> rightMost(pts.begin() + 2, pts.end());

        // now, sort the left-most coordinates according to their
        // y-coordinates so we can grab the top-left and bottom-left
        // points, respectively
        std::sort(leftMost.begin(), leftMost.end(), [](cv::Point2f a, cv::Point2f b) { return a.y < b.y; });
        cv::Point2f tl = leftMost[0];
        cv::Point2f bl = leftMost[1];

        // now, sort the right-most coordinates according to their
        // y-coordinates so we can grab the top-right and bottom-right
        // points, respectively
        std::sort(rightMost.begin(), rightMost.end(), [](cv::Point2f a, cv::Point2f b) { return a.y < b.y; });
        cv::Point2f tr = rightMost[0];
        cv::Point2f br = rightMost[1];

        // return the coordinates in top-left, top-right,
        // bottom-right, and bottom-left order
        std::vector<cv::Point2f> result{ tl, tr, br, bl };
        return result;
    }

    std::vector<cv::Mat> generate_sliding_masks(int image_size, int window_length) {

        int num_masks = image_size/window_length;
        std::vector<cv::Mat> masks(num_masks);

        for (int i=0; i<num_masks; i++) {
            masks[i] = cv::Mat::zeros(320, 320, CV_8U);
            masks[i].rowRange(i*window_length, (i+1)*window_length) = cv::Scalar(255);
        }

        return masks;
    }

/*
    void full_line(cv::Mat *img, cv::Point a, cv::Point b, cv::Scalar color){
        double slope = Slope(a.x, a.y, b.x, b.y);

        cv::Point p(0,0), q(img->cols,img->rows);

        p.y = -(a.x - p.x) * slope + a.y;
        q.y = -(b.x - q.x) * slope + b.y;

        line(*img,p,q,color,1,8,0);
    }
*/


    std::vector<cv::Rect> remove_potholes(cv::Mat* original_image) {
        std::vector<cv::Rect> potholes;
        // assume image is single channel

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(*original_image,contours,cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (contours.size() > 0 ) {
            for (int i=0; i<contours.size(); i++) {
                cv::Rect rect = cv::boundingRect(contours[i]);
                if (abs(rect.width - rect.height) <= 5) {
                    potholes.push_back(rect);

                    cv::Point pt1, pt2;
                    pt1.x = rect.x;
                    pt1.y = rect.y;
                    pt2.x = rect.x + rect.width;
                    pt2.y = rect.y + rect.height;

                    cv::rectangle(*original_image, pt1, pt2, cv::Scalar(0), -1);
                }

            }
        }

        return potholes;
    }

    cv::Mat add_potholes(cv::Mat* original_image, std::vector<cv::Rect> potholes) {
        for (int i=0; i<potholes.size(); i++) {

            cv::Point pt1, pt2;
            cv::Rect rect = potholes[i];
            pt1.x = rect.x;
            pt1.y = rect.y;
            pt2.x = rect.x + rect.width;
            pt2.y = rect.y + rect.height;

            cv::circle(*original_image, 0.5*(pt1+pt2), 45, cv::Scalar(255), -1);
        }

        return *original_image;
    }




    void full_line(cv::Mat* img, cv::Point a, cv::Point b, cv::Scalar color)
    {
        //points of line segment
        cv::Point p1 = a;
        cv::Point p2 = b;

        //points of line segment which extend the segment P1-P2 to
        //the image borders.
        cv::Point p,q;

        //test if line is vertical, otherwise computes line equation
        //y = ax + b
        if (p2.x == p1.x)
        {
            p = cv::Point(p1.x, 0);
            q = cv::Point(p1.x, img->rows);
        }
        else
        {
            double a = (double)(p2.y - p1.y) / (double) (p2.x - p1.x);
            double b =  p1.y - a*p1.x;

            p = cv::Point(0, b);
            q = cv::Point(img->rows, a*(img->rows) + b);

            //clipline to the image borders. It prevents a known bug on OpenCV
            //versions 2.4.X when drawing
            cv::clipLine(cv::Size(img->rows, img->cols), p, q);
        }

        // std::cout << abs(int(a.y - b.y)) << " =  thickness\n";
        cv::line(*img, p, q, color, 24);
        // cv::line(*img, p, q, color, 2);
    }

    double slope(cv::Point a, cv::Point b){
        return (double)(a.y-b.y)/(a.x-b.x);
    }


    std::vector<cv::Mat> extend_lanes(cv::Mat image, int num_masks, std::vector<cv::Mat> masks) {
        // std::cout << "[extend lanes]; input image size " << image.size() << " " << image.channels() << std::endl;



        std::vector<cv::Mat> results;
        for (int i = 0; i < num_masks; i++) {
            cv::Mat result;
            // std::cout << "applying bitwise and on image of size " << image.channels()
                // << " and mask of no channels " << masks[i].channels() << std::endl;
            // std::cout << "the types of the image and mask respectively are " << type2str(image.type()) << " " << type2str(masks[i].type()) << std::endl;


            cv::bitwise_and(image, masks[i], result);

            // std::cout << "now finding contours\n";
            auto contours = find_contours(result);
            if (contours.size() > 0) {
                // std::cout << "num contours is greater than 0\n";
                for (int j=0; j<contours.size(); j++) {
                    // auto blackbox = cv::minAreaRect(contours[i]);
                    // blackbox.points(vertices);

                    // std::cout << "finding the bounding rect\n";

                    cv::RotatedRect minRect = cv::minAreaRect(contours[j]);
                    std::vector<cv::Point2f> vertices(4);
                    minRect.points(vertices.data());

                    // std::cout << "points without sorting : " << vertices << "\n";
                    vertices = lane_extension::order_points_new(vertices);
                    // std::cout << "points with sorting : " << vertices << "\n";

                    cv::Point mid_point_a = 0.5*(vertices[0] + vertices[1]);
                    cv::Point mid_point_b = 0.5*(vertices[2] + vertices[3]);
                    int length1 = abs(vertices[0].y - vertices[3].y);
                    int breadth1 = abs(vertices[0].x - vertices[1].x);
                    int length2 = abs(vertices[1].y - vertices[2].y);
                    int breadth2 = abs(vertices[3].x - vertices[2].x);
                    int length = 0.5*(length1 + length2);
                    int breadth = 0.5*(breadth1 + breadth2);
                    // std::cout << "The length and breadth are " << length << " " << breadth << std::endl;

                    float angle;

                    if ((length >= 50) && (breadth >= 50) // ensure that the thingi is not random noise
                        && (abs(length-breadth) >= 15)) { // ensure that the bounding box is not a square i.e. pothole
                        if (breadth <= 130) {
                            angle = abs(atan(lane_extension::slope(mid_point_a, mid_point_b)))*180/3.14159;
                        } else if (breadth >= 130) {
                            angle = abs(90 - abs(atan(lane_extension::slope(mid_point_a, mid_point_b)))*180/3.14159);
                        } else {
                            angle = -1;
                        }
                    } else {
                        angle = -1;
                    }

                    // std::cout << "the angle is : " << angle << "\n";
                    if ((angle>=75)) {
                        lane_extension::full_line(&result,
                                                  0.5*(vertices[0]+vertices[1]),
                                                  0.5*(vertices[3]+vertices[2]),
                                                  // cv::Scalar(255, 255, 255));
                                                  cv::Scalar(255));
                    } else if (angle<=15 && angle >=0) {
                        lane_extension::full_line(&result,
                                                  0.5*(vertices[0]+vertices[3]),
                                                  0.5*(vertices[2]+vertices[1]),
                                                  // cv::Scalar(255, 255, 255));
                                                  cv::Scalar(255));
                    }
                }
            }
            results.push_back(result);
            // imshow("Mask " + std::to_string(i), result);
        }

        // Wait for a key press
        // cv::waitKey(0);
        return results;
    }

    cv::Mat integrate_imgs(cv::Mat original_img, cv::Mat mask, std::vector<cv::Mat> results) {
        int length = results.size();

        cv::Mat gray_img = original_img;
        // cv::cvtColor(original_img, gray_img, cv::COLOR_BGR2GRAY);

        cv::Mat final_image = gray_img;
/*
        // std::cout << "the gray image size is " << gray_img.size() << " " << gray_img.channels() << std::endl;
        std::cout << "the mask size is " << mask.size() << " " << mask.channels() << std::endl;
        std::cout << "the final image size is " << final_image.size() << " " << final_image.channels() << std::endl;
*/
        for(int i=0+length/3; i<length; i++) {
            cv::Mat temp = cv::Mat::zeros(cv::Size(gray_img.rows, gray_img.cols), 0);

            // std::cout << "performing bitwise and between result and mask, storing in temp\n";
            cv::Mat dst = cv::Mat::zeros(cv::Size(mask.rows, mask.cols), CV_8U);
            // cv::cvtColor(results[i], dst, cv::COLOR_BGR2GRAY);
            // std::cout << "the result image size is " << dst.size() << " " << dst.channels() << std::endl;
            // std::cout << "the size of temp is " << temp.size() << " " << temp.channels() << std::endl;
            cv::bitwise_and(results[i], mask, temp);
            // std::cout << "performing bitwise or between temp and gray_image, storing in temp\n";

            temp = temp | gray_img;
            // cv::bitwise_or(temp, gray_img, dst);
            // temp = dst;
            // std::cout << "performing bitwise or between temp and final_image, storing in final_image\n";
            cv::bitwise_or(final_image, temp, final_image);
        }
        return final_image;
    }

    cv::Mat process_lane_layer(cv::Mat lanes_layer, int window_length=80) {
        int image_size = lanes_layer.rows;

        int num_masks = image_size/window_length;

        std::vector<cv::Mat> masks(num_masks);
        masks = lane_extension::generate_sliding_masks(image_size, window_length);

        // find potholes add their positions to a vector
        // remove potholes from the original image
        // auto potholes = lane_extension::remove_potholes(&lanes_layer);

        // std::cout << "generated the sliding masks, now extending lanes\n";
        std::vector<cv::Mat> results = lane_extension::extend_lanes(lanes_layer, num_masks, masks);


        // std::cout << "lane extended images generated, now integrating all images\n";
        cv::Mat extension_mask = cv::Mat::zeros(cv::Size(lanes_layer.rows, lanes_layer.cols), CV_8U);
        extension_mask.rowRange(window_length, image_size) = cv::Scalar(255);

        cv::Mat final_img = lane_extension::integrate_imgs(lanes_layer, extension_mask, results);

        // lane_extension::add_potholes(&final_img, potholes);
        // TODO add potholes back to the final image

        return final_img;
    }

}

#endif // LANE_EXTENSION_H_
