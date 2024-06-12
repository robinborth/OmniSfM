#pragma once

#include <opencv2/opencv.hpp>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

struct Image
{
    int id;
    cv::Mat rgb;
    cv::Mat normal;
    cv::Mat depth;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
};

class ImageStorage
{
public:
    ImageStorage(const std::string &datasetDir) : datasetDir(datasetDir)
    {
        this->sift = cv::SIFT::create();
    };

    void loadImages()
    {
        if (!readFileList("rgb", filenameRGBImages, idRGBImages) || !loadImages("rgb", filenameRGBImages, idRGBImages))
            return;
        if (!readFileList("depth", filenameDepthImages, idDepthImages) || !loadImages("depth", filenameDepthImages, idDepthImages))
            return;
        if (!readFileList("normal", filenameNormalImages, idNormalImages) || !loadImages("normal", filenameNormalImages, idNormalImages))
            return;
    }

    void detectKeypoints()
    {
        for (auto &image : images)
        {
            sift->detectAndCompute(image.rgb, cv::noArray(), image.keypoints, image.descriptors);
        }
    }

    void drawKeypoints(u_int16_t id, std::string outputPath)
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

    std::vector<Image> images;

    Image *findImage(u_int16_t id)
    {
        auto it = std::find_if(images.begin(), images.end(), [id](Image &a)
                               { return a.id == id; });

        if (it != images.end())
            return &(*it);
        return nullptr;
    }

private:
    bool readFileList(const std::string &type, std::vector<std::string> &filenames, std::vector<uint16_t> &ids)
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

    bool checkLoadImage(cv::Mat image)
    {
        if (image.empty())
        {
            std::cerr << "Could not open or find the image!" << std::endl;
            return false;
        }
        return true;
    }

    bool loadImages(const std::string &type, std::vector<std::string> &filenames, std::vector<uint16_t> &ids)
    {
        for (size_t i = 0; i < filenames.size(); ++i)
        {
            uint16_t id = ids[i];
            Image *img = findImage(id);

            if (!img) // not found create a new image
            {
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
            if (type == "normal")
            {
                cv::Mat normalImage = cv::imread(filenames[i], cv::IMREAD_COLOR);
                img->normal = normalImage;
                if (!checkLoadImage(normalImage))
                    return false;
            }
            if (type == "depth")
            {
                cv::Mat depthImage = cv::imread(filenames[i], cv::IMREAD_ANYDEPTH);
                img->depth = depthImage;
                if (!checkLoadImage(depthImage))
                    return false;
            }
        }
        return true;
    }

    std::string datasetDir;
    cv::Ptr<cv::SIFT> sift;
    std::vector<std::string> filenameDepthImages;
    std::vector<uint16_t> idDepthImages;
    std::vector<std::string> filenameNormalImages;
    std::vector<uint16_t> idNormalImages;
    std::vector<std::string> filenameRGBImages;
    std::vector<uint16_t> idRGBImages;
};