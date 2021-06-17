
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/core.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

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
void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait, std::string nomeImg)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));
    int i =0;
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
    std::string windowName = "3D Objects";
    //cv::namedWindow(windowName, 1);
    //cv::resize(topviewImg, topviewImg, cv::Size(), 0.3, 0.3);
    //cv::imshow(windowName, topviewImg);

    string nomeimg = nomeImg + "_" + std::to_string(i) +".png";
    cv::imwrite(nomeimg, topviewImg);
    i++;

    //if(bWait)
    //{
    //    cv::waitKey(0); // wait for key to be pressed
    //}
}

void SmallerBox(cv::Rect &currSmallerBox, cv::Rect ROI, float shrinkFactor)
{
    currSmallerBox.x = ROI.x + shrinkFactor * ROI.width / 2.0;
    currSmallerBox.y = ROI.y + shrinkFactor * ROI.height / 2.0;
    currSmallerBox.width = ROI.width * (1 - shrinkFactor);
    currSmallerBox.height = ROI.height * (1 - shrinkFactor);
}

// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    double threshold = 1.0;
    
    std::vector<cv::KeyPoint> insideROIkpts;
    std::vector<cv::DMatch> insideROIdmatch;
    std::vector<double> distances;
    
    // For each keypoint match in the current bounding box, compute the distance between matched keypoints position and insert into a vector.
    for (auto match = kptMatches.begin(); match != kptMatches.end(); match++)
    {
        // shrink current bounding box slightly to avoid having too many outlier points around the edges
        cv::Rect currSmallerBox;
        SmallerBox(currSmallerBox, boundingBox.roi, 0.2);

        if (currSmallerBox.contains(kptsCurr[match->trainIdx].pt))
        {
            double dist = cv::norm(kptsPrev[match->queryIdx].pt - kptsCurr[match->trainIdx].pt);            
            distances.push_back(dist);
            insideROIkpts.push_back(kptsCurr[match->trainIdx]);
            insideROIdmatch.push_back(*match);
            
        }
    }
    
    if (insideROIkpts.size()>2)
    {
    	// Find the mean and standart deviation
        double dist_mean = std::accumulate(distances.begin(), distances.end(), 0.0)/distances.size();
        double dist_std = 0;
        for(auto it=distances.begin(); it!=distances.end(); ++it) {
            dist_std += std::pow(*it-dist_mean, 2);
        }
        dist_std = std::sqrt(dist_std/distances.size());

	// Find the Zscore for each distance in the vector, and filter the keypoints with the respective distance Zscore less than a threshold value (1.0). 
        float Zscore = 0.0;
        for(int i = distances.size()-1; i > 0; i--) {
            Zscore = (distances[i]-dist_mean)/dist_std;
            
            if (Zscore>threshold && insideROIkpts.size()>2)
            {
                insideROIkpts.erase(insideROIkpts.begin()+i);
                insideROIdmatch.erase(insideROIdmatch.begin()+i);
            }
        }
    }
    
    boundingBox.keypoints = insideROIkpts;
    boundingBox.kptMatches = insideROIdmatch;
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    
    // compute distance ratios between all matched keypoints
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer kpt. loop
        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { // inner kpt.-loop
            
            double minDist = 100.0; // min. required distance
            // compute distances and distance ratios
            double distCurr = cv::norm(kptsCurr.at(it1->trainIdx).pt - kptsCurr.at(it2->trainIdx).pt);
            double distPrev = cv::norm(kptsPrev.at(it1->queryIdx).pt - kptsPrev.at(it2->queryIdx).pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist && distPrev >= 0)
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
    }
    else{

        // STUDENT TASK (replacement for meanDistRatio)
        std::sort(distRatios.begin(), distRatios.end());
        long medIndex = floor(distRatios.size() / 2.0);
        double medDistRatio = distRatios.size() % 2 == 0 ? (distRatios[medIndex - 1] + distRatios[medIndex]) / 2.0 : distRatios[medIndex]; // compute median dist. ratio to remove outlier influence

        float dT = 1 / frameRate;
        TTC = -dT / (1 - medDistRatio);
        // EOF STUDENT TASK
    }
}


void filterLidarOutliers(std::vector<LidarPoint> &points)
{
    // Filter Lidar points with distance in x axis near to the median value (median-threshold <= filtered <= median+threshold).
    float threshold = 0.1;

    std::vector<float> distances;
    for(auto it=points.begin(); it!=points.end(); ++it)
        distances.push_back(it->x);

    std::sort(distances.begin(), distances.end());
    
    int index = distances.size()/2;
    float median = distances[index];

    for(auto it=points.begin(); it!=points.end(); ++it) {
        float dist = std::abs(it->x-median);
        if (dist > threshold)
        {
            points.erase(it);
            it--;
        }
    }
}

void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    // auxiliary variables
    double dT = 1.0/frameRate; // time between two measurements in seconds

    // filter outliers
    filterLidarOutliers(lidarPointsCurr);
    filterLidarOutliers(lidarPointsPrev);
    
    // find closest distance in filtered Lidar points 
    double minXPrev = 1e9, minXCurr = 1e9;
    for(auto it=lidarPointsPrev.begin(); it!=lidarPointsPrev.end(); ++it) {
        minXPrev = minXPrev>it->x ? it->x : minXPrev;
    }
    
    for(auto it=lidarPointsCurr.begin(); it!=lidarPointsCurr.end(); ++it) {
        minXCurr = minXCurr>it->x ? it->x : minXCurr;
    }
    
    // compute TTC from both measurements
    TTC = minXCurr * dT / (minXPrev-minXCurr);
    
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    std::map<std::pair<int,int>,int> bbPairs;
    
    // Get all possible bounding box pairs (one from current image and the other from the previous image) and the number of matched keypoints inside both bounding boxes for each pair.
    for (auto currBB = currFrame.boundingBoxes.begin(); currBB != currFrame.boundingBoxes.end(); currBB++)
    {
        for (auto match = matches.begin(); match != matches.end(); match++)
        {
            cv::KeyPoint currKpt = currFrame.keypoints[match->trainIdx];
            cv::KeyPoint prevKpt = prevFrame.keypoints[match->queryIdx];

            if (currBB->roi.contains(currKpt.pt))
            {
                for (auto prevBB = prevFrame.boundingBoxes.begin(); prevBB != prevFrame.boundingBoxes.end(); prevBB++)
                {
                    if (prevBB->roi.contains(prevKpt.pt))
                    {
                    	 // Count the number of matched keypoints inside bounding box for each pair.
                        std::pair<int,int> candidatePair(prevBB->boxID,currBB->boxID);
                        if(bbPairs.find(candidatePair) == bbPairs.end())
                        {
                            bbPairs.insert(std::pair<std::pair<int,int>,int>(candidatePair,1));
                        }
                        else
                        {
                            bbPairs.at(candidatePair)++;
                        }
                    }
                }
            }
        }
    }
    
    // Sort all the pairs by the number of keypoints matches
    std::vector<std::pair<std::pair<int,int>,int>> bbPairsList;
    for(auto bbPair = bbPairs.begin()++; bbPair != bbPairs.end(); bbPair++)
    {
        bbPairsList.push_back(std::pair<std::pair<int,int>,int>(bbPair->first, bbPair->second)); 
    }
    auto compare = [](std::pair<std::pair<int,int>,int> const & a, std::pair<std::pair<int,int>,int> const & b)
    {         
     return a.second != b.second?  a.second > b.second : a.first.second > b.first.second;
    };
    std::sort(bbPairsList.begin(), bbPairsList.end(), compare);

    // Insert the pairs with highest number of keypoint matches to bbBestMatches without repeating already used bounding boxes along the choosed pairs.
    bbBestMatches.insert(bbPairsList.begin()->first);
    for (auto bbPair = bbPairsList.begin()++; bbPair != bbPairsList.end(); bbPair++)
    {
        bool found = false;
        for (auto bbBestMatch = bbBestMatches.begin()++; bbBestMatch != bbBestMatches.end(); bbBestMatch++)
        {
            if ((bbPair->first.first==bbBestMatch->first)||(bbPair->first.second==bbBestMatch->second))
            {
                found = true;
            }
        }
        if (!found)
        {
            bbBestMatches.insert(bbPair->first);

            // Display bounding boxes
            if(false)
            {
                int currID = -1;
                for (int i=0; (i<currFrame.boundingBoxes.size())&&(currID<0); i++)
                {
                    currID = currFrame.boundingBoxes[i].boxID==bbPair->first.second ? i : -1;
                }
                int prevID = -1;
                for (int i=0; (i<prevFrame.boundingBoxes.size())&&(prevID<0); i++)
                {
                    prevID = prevFrame.boundingBoxes[i].boxID==bbPair->first.first ? i : -1;
                }
                if (currID>=0 && prevID>=0)
                {
                    cv::Rect currBox = currFrame.boundingBoxes[currID].roi;
                    cv::Rect prevBox = prevFrame.boundingBoxes[prevID].roi;

                    cv::Mat visImg = currFrame.cameraImg.clone();
                    cv::rectangle(visImg, cv::Point(currBox.x, currBox.y), cv::Point(currBox.x + currBox.width, currBox.y + currBox.height), cv::Scalar(0, 255, 0), 2);
                    cv::imshow("windowName", visImg);
                    cv::waitKey(0); // wait for key to be pressed

                    visImg = prevFrame.cameraImg.clone();
                    cv::rectangle(visImg, cv::Point(prevBox.x, prevBox.y), cv::Point(prevBox.x + prevBox.width, prevBox.y + prevBox.height), cv::Scalar(0, 255, 0), 2);
                    cv::imshow("windowName", visImg);
                    cv::waitKey(0); // wait for key to be pressed
                }
            }
        }        
    }

    // Filter keypoint from matches that are out of the current or previous image bounding boxes, 
    for (auto match = matches.begin(); match != matches.end(); match++)
    {
        cv::KeyPoint currKpt = currFrame.keypoints[match->trainIdx];
        cv::KeyPoint prevKpt = prevFrame.keypoints[match->queryIdx];

        int currID = -1;
        for (auto currBB=currFrame.boundingBoxes.begin(); (currBB!=currFrame.boundingBoxes.end())&&(currID<0); currBB++)
        {
            currID = currBB->roi.contains(currKpt.pt) ? currBB->boxID : -1;
        }
        int prevID = -1;
        for (auto prevBB=prevFrame.boundingBoxes.begin(); (prevBB!=prevFrame.boundingBoxes.end())&&(prevID<0); prevBB++)
        {
            prevID = prevBB->roi.contains(prevKpt.pt) ? prevBB->boxID : -1;
        }
        if (currID>=0 && prevID>=0)
        { 
            int correctCurrID = bbBestMatches[prevID];
            if (currID != correctCurrID)
            {
                matches.erase(match);
                match--;
            }
        }
        else
        {
            matches.erase(match);
            match--;
        }
    }
}
