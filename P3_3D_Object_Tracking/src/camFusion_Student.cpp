
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        // pixel coordinates
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0); 
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0); 

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}

/* 
* The show3DObjects() function below can handle different output image sizes, but the text output has been manually tuned to fit the 2000x2000 size. 
* However, you can make this function work for other sizes too.
* For instance, to use a 1000x1000 size, adjusting the text positions by dividing them by 2.
*/
void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    cv::KeyPoint currKpt, prevKpt;
    vector<cv::DMatch> matchesInBoundingBox;
    vector<double>euclideanDist;

    // Take matches in the boundingBox
    for (auto it = kptMatches.begin(); it != kptMatches.end(); ++it)
    {
        currKpt = kptsCurr[(*it).trainIdx];
        prevKpt = kptsPrev[(*it).queryIdx];

        if(boundingBox.roi.contains(currKpt.pt))
        {
            matchesInBoundingBox.push_back((*it));
            euclideanDist.push_back(cv::norm(currKpt.pt-prevKpt.pt));  // Distance of the currKpt and the prevKpt
        }
    }
    
    // Remove outliers
    // compute mean of matches Euclidean distance
    double meanDist = std::accumulate(euclideanDist.begin(), euclideanDist.end(), 0.0) / euclideanDist.size();
    double threshDist = 1.5 * meanDist;

    // associate best matches to the boundingBox
    auto it1= matchesInBoundingBox.begin();
    auto it2 = euclideanDist.begin();
    for ( ; it2 != euclideanDist.end(); ++it2,++it1)
    {
        if((*it2) <  threshDist)
        {
            currKpt = kptsCurr[(*it1).trainIdx];
            boundingBox.kptMatches.push_back((*it1));
            boundingBox.keypoints.push_back(currKpt);
        }
    }
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // compute distance ratios between all matched keypoints
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer kpt. loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { // inner kpt.-loop

            double minDist = 100.0; // min. required distance

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { // avoid division by zero

                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }

    std::sort(distRatios.begin(), distRatios.end());
    long medIndex = floor(distRatios.size() / 2.0);
    double medDistRatio = distRatios.size() % 2 == 0 ? (distRatios[medIndex - 1] + distRatios[medIndex]) / 2.0 : distRatios[medIndex]; // compute median dist. ratio to remove outlier influence

    double dT = 1. / frameRate;
    TTC = -dT / (1 - medDistRatio);
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    bool useMin = false; // if false, useAverage
    if (useMin) 
    {
        // find closest distance to Lidar points within ego lane
        double minXPrev = 1e9, minXCurr = 1e9;
        for (auto prev_point: lidarPointsPrev) {
            minXPrev = std::min(minXPrev, prev_point.x);
        }
        for (auto prev_point: lidarPointsCurr) {
            minXCurr = std::min(minXCurr, prev_point.x);
        }
        
        if (minXPrev <= minXCurr) 
        {
            TTC = NAN;
        }
        else
        {
            TTC = minXCurr * (1. / frameRate) / (minXPrev - minXCurr);
        }
    }
    else
    {   
        auto lambda = [&](LidarPoint point){return point.x;};
        double avgXPrev = 0., avgXCurr = 0.;
        for (LidarPoint point : lidarPointsPrev) {
            avgXPrev += point.x / lidarPointsPrev.size();
        }
        for (LidarPoint point : lidarPointsCurr) {
            avgXCurr += point.x / lidarPointsCurr.size();
        }

        if (avgXPrev <= avgXCurr) 
        {
            TTC = NAN;
        }
        else
        {
            TTC = avgXCurr * (1. / frameRate) / (avgXPrev - avgXCurr);
        }
    }
    
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    // https://docs.opencv.org/4.1.0/db/d39/classcv_1_1DescriptorMatcher.html#a0f046f47b68ec7074391e1e85c750cba
    // https://docs.opencv.org/4.1.0/d4/de0/classcv_1_1DMatch.html

    int prev_num_boxes = prevFrame.boundingBoxes.size();
    int curr_num_boxes = currFrame.boundingBoxes.size();
    int counts[prev_num_boxes][curr_num_boxes] = {};
    
    for (auto match : matches)     
    {
        cv::KeyPoint query = prevFrame.keypoints[match.queryIdx];
        cv::KeyPoint train = currFrame.keypoints[match.trainIdx];
        bool query_found = false, train_found = false;
    	int query_id, train_id;

        for (int i = 0; i < prevFrame.boundingBoxes.size(); i++) {
    		if (prevFrame.boundingBoxes[i].roi.contains(cv::Point(query.pt.x, query.pt.y))) {
    			query_found = true;
    			query_id = i;
    			break;
    		}
    	}
        for (int i = 0; i < currFrame.boundingBoxes.size(); i++) {
    		if (currFrame.boundingBoxes[i].roi.contains(cv::Point(train.pt.x, train.pt.y))) {
    			train_found = true;
    			train_id = i;
    			break;
    		}
    	}
        if (query_found && train_found) {
            counts[query_id][train_id] += 1;
    	}
    }

    for (int i = 0; i < prev_num_boxes; i++)
    {  
        int max_count = 0;
        int max_id = -1;
        for (int j = 0; j < curr_num_boxes; j++) {
            if (counts[i][j] > max_count)
            {  
                max_count = counts[i][j];
                max_id = j;
            }
        }
        bbBestMatches[i] = max_id;
    } 
}
