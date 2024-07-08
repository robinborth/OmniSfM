#include "ImageStorage.h"

#include <iostream>
#include <fstream>
#include "Settings.h"
#include <opencv2/core/eigen.hpp>

ImageStorage::ImageStorage(const Settings &settings)
{
    this->datasetDir = settings.rootDir + "/data/" + settings.dataset + "/";
    this->sift = cv::SIFT::create(settings.siftNumFeatures);
};

void ImageStorage::loadImages()
{
    if (!readFileList("rgb", filenameRGBImages, idRGBImages) || !loadImages("rgb", filenameRGBImages, idRGBImages))
        return;
    if (!readFileList("depth", filenameDepthImages, idDepthImages) || !loadImages("depth", filenameDepthImages, idDepthImages))
        return;
    // if (!readIntrinsics())
    //     return;
    readIntrinsics();
    // Optional: try to read the extrinsics but they don't need to be provided extrinsics.txt
    readExtrinsics();
};

void ImageStorage::detectKeypoints()
{
    for (auto &image : images)
    {
        sift->detectAndCompute(image.rgb, cv::noArray(), image.keypoints, image.descriptors);
    }
};

void ImageStorage::drawKeypoints(int64_t id, std::string outputPath)
{
    // Find image with the id
    Image *image = findImage(id);
    if (!image)
    {
        std::cerr << "Error: Could not find image." << std::endl;
        return;
    }
    // Draw the keypoints
    cv::Mat img_keypoints;
    cv::drawKeypoints(image->rgb, image->keypoints, img_keypoints);
    // Save the result
    cv::imwrite(outputPath, img_keypoints);
}

Image *ImageStorage::findImage(int64_t id)
{
    auto it = std::find_if(images.begin(), images.end(), [id](Image &a)
                           { return a.id == id; });

    if (it != images.end())
        return &(*it);
    return nullptr;
}

bool ImageStorage::readIntrinsics()
{
    std::string filePath = datasetDir + "intrinsics.txt";
    std::ifstream fileList(filePath, std::ios::in);
    if (!fileList.is_open())
    {
        std::cerr << "Error: Could not open the file " << filePath << std::endl;
        return false;
    }
    // skip the header
    std::string dump;
    std::getline(fileList, dump);
    while (fileList.good())
    {
        int64_t id;
        fileList >> id;
        float fX;
        fileList >> fX;
        float fY;
        fileList >> fY;
        float cX;
        fileList >> cX;
        float cY;
        fileList >> cY;
        Image *img = findImage(id);
        if (!img) // not found create a new image
        {
            std::cout << "At the point for attaching the intrinsics we need to have created the image with id:" << id << std::endl;
            return false;
        }
        img->K << fX, 0.0f, cX,
            0.0f, fY, cY,
            0.0f, 0.0f, 1.0f;
    }
    fileList.close();
    return true;
}

bool ImageStorage::readExtrinsics()
{
    std::string filePath = datasetDir + "extrinsics.txt";
    std::ifstream fileList(filePath, std::ios::in);
    if (!fileList.is_open())
    {
        std::cerr << "No extrinsics.txt groundtruth file found." << filePath << std::endl;
        return false;
    }
    // skip the header
    std::string dump;
    std::getline(fileList, dump);
    while (fileList.good())
    {
        int64_t id;
        fileList >> id;

        float tx;
        fileList >> tx;
        float ty;
        fileList >> ty;
        float tz;
        fileList >> tz;
        float qx;
        fileList >> qx;
        float qy;
        fileList >> qy;
        float qz;
        fileList >> qz;
        float qw;
        fileList >> qw;

        Eigen::Vector3f translation = {tx, ty, tz};
        Eigen::Quaternionf rot = {qx, qy, qz, qw};
        Eigen::Matrix4f transf;
        transf.setIdentity();
        transf.block<3, 3>(0, 0) = rot.toRotationMatrix();
        transf.block<3, 1>(0, 3) = translation;
        if (rot.norm() == 0)
        {
            std::cout << "The norm is 0!" << std::endl;
            return false;
        }
        // we need to invert the transformation matrix
        // to go
        transf = transf.inverse().eval();

        Image *img = findImage(id);
        if (!img) // not found create a new image
        {
            std::cout << "At the point for attaching the extrinsics we need to have created the image with id:" << id << std::endl;
            return false;
        }
        cv::Vec4f q(qx, qy, qz, qw);
        Eigen::Matrix3f r2 = transf.block<3, 3>(0, 0);
        Eigen::Vector3f t2 = transf.block<3, 1>(0, 3);
        cv::Mat R;
        cv::Mat T;
        cv::eigen2cv(r2, R);
        cv::eigen2cv(t2, T);
        img->t = T;
        img->R = R;
        img->P = transf;
    }
    fileList.close();
    return true;
}

bool ImageStorage::readFileList(const std::string &type, std::vector<std::string> &filenames, std::vector<int64_t> &ids)
{
    std::string filePath = datasetDir + type + ".txt";
    std::ifstream fileDepthList(filePath, std::ios::in);
    if (!fileDepthList.is_open())
    {
        std::cerr << "Error: Could not open the file " << filePath << std::endl;
        return false;
    }
    // skip the header
    filenames.clear();
    ids.clear();
    std::string dump;
    std::getline(fileDepthList, dump);
    while (fileDepthList.good())
    {
        uint16_t id;
        fileDepthList >> id;
        std::string filename;
        fileDepthList >> filename;
        if (filename == "")
            break;
        filenames.push_back(datasetDir + filename);
        ids.push_back(id);
    }
    fileDepthList.close();
    return true;
}

bool ImageStorage::checkLoadImage(cv::Mat &image)
{
    if (image.empty())
    {
        std::cerr << "Could not open or find the image!" << std::endl;
        return false;
    }
    return true;
}

bool ImageStorage::loadImages(const std::string &type, std::vector<std::string> &filenames, std::vector<int64_t> &ids)
{
    for (size_t i = 0; i < filenames.size(); i++) // change the data folder, e.g. freiburg_full
    {
        int64_t id = ids[i];
        std::cout << "Loading image with id " << id << std::endl;
        Image *img = findImage(id);

        if (!img) // not found create a new image
        {
            std::cout << "Creating new image with id " << id << std::endl;
            images.push_back(Image{id});
            img = &images.back();
        }

        // save the image with specific preprocessing for image types
        if (type == "rgb")
        {
            cv::Mat rgbImage = cv::imread(filenames[i], cv::IMREAD_COLOR);
            img->rgb = rgbImage;
            if (!checkLoadImage(rgbImage))
                return false;
        }
        if (type == "depth")
        {
            cv::Mat depthImage = cv::imread(filenames[i], cv::IMREAD_UNCHANGED);
            // Check if the image is loaded successfully
            if (depthImage.empty())
                std::cerr << "Error: Unable to open depth image!" << std::endl;

            // Convert to float32 if the depth image is not already in this format
            if (depthImage.type() != CV_32F)
                depthImage.convertTo(depthImage, CV_32F, 1.0 / 65535.0);

            // converts and stores the depth image in the image struct
            cv::cv2eigen(depthImage, img->depth);

            if (!checkLoadImage(depthImage))
                return false;
        }

        img->w = 1.0;
        img->q = 0.0;
    }
    std::cout << "Loaded images: " << images.size() << std::endl;
    return true;
}
