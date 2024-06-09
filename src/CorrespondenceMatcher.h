#include <opencv2/opencv.hpp>

class CorrespondenceMatcher
{
public:
    CorrespondenceMatcher()
    {
        this->sift = cv::SIFT::create();
    }

    void load_keypoints(std::string imagePath)
    {
        // load the image from disk
        cv::Mat image = cv::imread(imagePath, cv::IMREAD_COLOR);
        // Check if the image is loaded properly
        if (image.empty())
        {
            std::cerr << "Could not open or find the image!" << std::endl;
            return;
        }
        // extract the keypoints with the features
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        sift->detectAndCompute(image, cv::noArray(), keypoints, descriptors);
    }

    void constructMatches() {}

    void selectMatches() {}

private:
    cv::Ptr<cv::SIFT> sift;
};