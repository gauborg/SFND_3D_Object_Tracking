
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
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        // creating 3D objects...
        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        // looping through all the bounding boxes ...
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

    } // eof loop over all Lidar point data
}


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
    // ...
    // we check if the current region of interest of bounding box contains the matched keypoints
    
    for (auto &match : kptMatches)
    {
        const auto &currKeyPoint = kptsCurr[match.trainIdx].pt;
        if (boundingBox.roi.contains(currKeyPoint))
        {
            boundingBox.kptMatches.push_back(match);
        }
    }

    double sums_of_distances = 0;

    // Remove outlier matches based on the euclidean distance between them in relation to all the matches in the bounding box.
    for (auto &it : boundingBox.kptMatches)
    {
        cv::KeyPoint CurrentKpt = kptsCurr.at(it.trainIdx); //get the keypoint from previous frame
        cv::KeyPoint PreviousKpt = kptsPrev.at(it.queryIdx);    //get the keypoint from current frame

        // use norm function from opencv to get the euclidean distance between two points
        double dist = cv::norm(CurrentKpt.pt - PreviousKpt.pt);
        sums_of_distances = sums_of_distances + dist;

    }
    // calculating the mean of distances of all keypoints matches

    double mean = sums_of_distances / boundingBox.kptMatches.size();
    double ratio = 1.5; //threshold for ratio
    
    for (auto it = boundingBox.kptMatches.begin(); it < boundingBox.kptMatches.end();)
    {
        cv::KeyPoint CurrentKpt = kptsCurr.at(it->trainIdx);
        cv::KeyPoint PreviousKpt = kptsPrev.at(it->queryIdx);
        
        double dist = cv::norm(CurrentKpt.pt - PreviousKpt.pt);

        if (dist >= mean*ratio)
        {
            boundingBox.kptMatches.erase(it);
        }
        else
        {
            it++;
        }
    }
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // ...
    
    double dT = 1.0/frameRate;

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

    // calculate median value index
    std::sort(distRatios.begin(), distRatios.end());
    int medianIndex = floor(distRatios.size() / 2.0);

    double medDistRatio;
    
    if (distRatios.size() % 2 == 0)
    {
        medDistRatio = (distRatios[medianIndex - 1] + distRatios[medianIndex]) / 2.0;
    }
    else
    {
        medDistRatio = distRatios[medianIndex];
    }

    TTC = -dT / (1 - medDistRatio);

    bool print_ttc_cam = true;
    if(print_ttc_cam)
    {
        cout<<"TTC using Camera = "<<TTC<<" seconds."<<endl<<endl;
    }
    
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    // ...
    double dT = 1.0/frameRate;  //time between two frames, here equal to framerate
    double minXPrev = 1e9;  //initializing to high values//
    double minXCurr = 1e9;

    //apply filter to remove outliers from noise/erroneous measurements
    
    // step 1. We first remove the points with y coordinate outside +ve or -ve 2.0 and with reflectivity < 0.1.
    /*
    for(auto pt_prev = lidarPointsPrev.begin(); pt_prev<lidarPointsPrev.end(); ++pt_prev)
    {
        if(fabs(pt_prev->y) >= 2.0 || (pt_prev->r < 0.1))        // here lanewidth = 4.0, hence removing any points beyond y = 2.0 or -2.0
        {
            lidarPointsPrev.erase(pt_prev);
        }
    }
    
    for(auto pt_curr = lidarPointsCurr.begin(); pt_curr<lidarPointsCurr.end(); ++pt_curr)
    {
        if(fabs(pt_curr->y) >= 2.0 || (pt_curr->r < 0.1))        // here lanewidth = 4.0, hence removing any points beyond y = 2.0 or -2.0
        {
            lidarPointsCurr.erase(pt_curr);
        }
    }
    */
    
    // display number of filtered points //

    //cout<<"Filtered Lidar Points in 1st step previous frame = "<<lidarPointsPrev.size()<<endl;
    //cout<<"Filtered Lidar Points in 1st step current frame = "<<lidarPointsCurr.size()<<endl;

    // step 2. we now add the x coordinates of all filtered points to calculate mean in the next step
    double x_total_prev = 0;
    double x_total_curr = 0;

    for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
    {
        //cout<<"x = "<<it->x<<", y = "<<it->y<<", z = "<<it->z<<endl;
        x_total_prev = x_total_prev + it->x;
    }
    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    {
        x_total_curr = x_total_curr + it->x;
    }

    // step 3. we calculate the mean x coordinate of filtered points
    
    double x_mean_prev = x_total_prev/lidarPointsPrev.size();
    //cout<<"x mean = "<<x_mean_prev<<endl;
    double x_mean_curr = x_total_curr/lidarPointsCurr.size();

    // step 4. remove points which seem like outliers - REMOVING POINTS WHICH DEVIATE MORE THAN 3% FROM MEAN X COORDINATE VALUE
    for(auto pt_prev = lidarPointsPrev.begin(); pt_prev<lidarPointsPrev.end(); ++pt_prev)
    {
        if(fabs(x_mean_prev - pt_prev->x) >= 0.03*x_mean_prev)
        {
            lidarPointsPrev.erase(pt_prev);
        }
    }
    for(auto pt_curr = lidarPointsCurr.begin(); pt_curr<lidarPointsCurr.end(); ++pt_curr)
    {
        if(fabs(x_mean_curr - pt_curr->x) >= 0.03*x_mean_curr)
        {
            lidarPointsCurr.erase(pt_curr);
        }
    }

    //step 5. Again display number of points

    //cout<<"Filtered Lidar Points in 2nd step previous frame = "<<lidarPointsPrev.size()<<endl;
    //cout<<"Filtered Lidar Points in 2nd step current frame = "<<lidarPointsCurr.size()<<endl;

    // step 6. again calculate total value of x coordinates
    double x_total_prev2 = 0;
    double x_total_curr2 = 0;

    for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
    {
        //cout<<"x = "<<it->x<<", y = "<<it->y<<", z = "<<it->z<<endl;
        x_total_prev2 = x_total_prev2 + it->x;
    }

    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    {
        x_total_curr2 = x_total_curr2 + it->x;
    }

    //step 7. now calculate mean again, mean values are updated

    x_mean_prev = x_total_prev2/lidarPointsPrev.size();
    x_mean_curr = x_total_curr2/lidarPointsCurr.size();

    /*
    for(auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
    {
        minXPrev = minXPrev >it->x ? it->x : minXPrev;
    }
    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    {
        minXCurr = minXCurr >it->x ? it->x : minXCurr;
    }
    double d0 = minXPrev;
    double d1 = minXCurr;
    */

    double d0 = x_mean_prev;
    double d1= x_mean_curr;

    // compute TTC from both measurements
    TTC = d0 * dT / (d0 - d1);

    bool print_ttc_value = true;
    if(print_ttc_value)
    {
        cout<<"TTC using Lidar = "<<TTC<<" seconds."<<endl;
    }

}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    // ...
    // Gaurav Borgaonkar Implementation
    
   for (auto &prevBox: prevFrame.boundingBoxes)   //iterating through all initial frames
   {
       //create an map variable here to store matches for every loop...
       map<int, int> matched_pairs;

       for(auto &currBox:currFrame.boundingBoxes) //iterating through every current frame
       {
           //cout<<"test_run";
           for(auto &match:matches) //iterating through all keypoint descriptors match pairs
           {
               auto &prev_kpt = prevFrame.keypoints[match.queryIdx].pt;     // from opencv documentation, queryidx is used to get the descriptor index
               auto prevBox_roi = prevBox.roi;
               if(prevBox_roi.contains(prev_kpt))   //checking if the current box region of interest contains matched keypoint descriptor
               {
                   auto &curr_kpt = currFrame.keypoints[match.trainIdx].pt; // trainIdx is opencv match function for train set of descriptors
                   auto currBox_roi = currBox.roi;
                   if(currBox_roi.contains(curr_kpt))
                   {
                        if(matched_pairs.count(currBox.boxID) == 0)
                        {
                            matched_pairs[currBox.boxID] = 1;
                        }
                        else
                        {
                            matched_pairs[currBox.boxID]++;
                        }
                   }
               }    //end of loop for mat
           }    //end of matches loop pairs
       }    //end of current frames loop

        // here we get the best match possible based on maximum number of keypoint matches
        
        auto bestMatch = std::max_element(matched_pairs.begin(), matched_pairs.end(), [](const std::pair<int, int> &a1, const std::pair<int, int> &a2) { return a1.second < a2.second; });
        
        bbBestMatches[prevBox.boxID] = bestMatch->first;

        //cout << "Bounding box matches: " << prevBox.boxID << " -> " << bestMatch->first << "\n";

   }

}


