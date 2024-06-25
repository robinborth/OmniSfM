#include "ImageStorage.h"

#include <iostream>
#include <fstream>
#include "Settings.h"



ImageStorage::ImageStorage(const Settings &settings)
{
    this->datasetDir = settings.rootDir + "/data/" + settings.dataset + "/";
    this->sift = cv::SIFT::create();
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

void ImageStorage::createPointCloudFromImage(const Image& img, Visualization& visualization) {
    cv::Mat depth = img.depth;
    cv::Mat K = img.K;
    cv::Mat R = img.R;
    cv::Mat t = img.t;

    if (img.depth.empty())
        return;
    if (img.K.empty() || img.R.empty() || img.t.empty())
        return;

    // Camera intrinsics
    // focal length
    float fx = K.at<float>(0, 0);
    float fy = K.at<float>(1, 1);
    // center point
    float cx = K.at<float>(0, 2);
    float cy = K.at<float>(1, 2);

    std::vector<cv::Point3f> points3D;
    std::vector<cv::Vec3b> colors;

    for (size_t v = 0; v < depth.rows; ++v) {
        for (size_t u = 0; u < depth.cols; ++u) {
            float Z = depth.at<float>(v, u);
            if (Z == 0) continue; // Skip invalid depth

            // Pixel Coordinates to Camera Coordinates
            float X = (u - cx) * Z / fx;
            float Y = (v - cy) * Z / fy;
            //std::cout << "pixel to camera coordinates" << std::endl;
            // Camera Coordinates to World Coordinates
            // Convert point to homogeneous coordinates
            cv::Mat P_camera = (cv::Mat_<float>(3, 1) << X, Y, Z);
            //std::cout << "homogenous coordinates" << std::endl;
            //std::cout << P_camera.size  << " " << P_camera.type() << std::endl;
            //std::cout << t.size << " " << t.type() << std::endl;
            //std::cout << R.size << " " << R.type() << std::endl;
            // Transform to world coordinates
            cv::Mat P_world = R * P_camera + t;
            //std::cout << "transform to world" << std::endl;
            points3D.emplace_back(P_world.at<double>(0, 0), P_world.at<double>(1, 0), P_world.at<double>(2, 0));

            // Add color
            cv::Vec3b color = img.rgb.at<cv::Vec3b>(v, u);
            //std::cout << "adding color" << std::endl;
            colors.push_back(color);
        }
    }
    //std::cout << "got points from depht map" << std::endl;
    visualization.addVertex(points3D, colors);
}

void ImageStorage::processImagesForPointCloud() {
    Visualization visualization("output_ImageStorage");
    std::vector<Eigen::Matrix4f> cameraPoses;
    std::vector<cv::Mat> intrinsics;

    for (const auto& img : images) {
        // Create point cloud from image
        createPointCloudFromImage(img, visualization);

        //std::cout << "adding camera to pc" << std::endl;
        // Convert camera pose to Eigen format and add to the list
        Eigen::Matrix4f eigenPose = SfMInitializer::combineRotationAndTranslationIntoMatrix(img.R, img.t);

        /*std::cout << "converted to Eigen: " << eigenPose << std::endl;
        std::cout << "img.R: " << img.R << std::endl << std::endl;
        std::cout << "img.t: " << img.t << std::endl << std::endl;*/
        cameraPoses.push_back(eigenPose);
        intrinsics.push_back(img.K);
    }
    // Add camera poses to the visualization
    visualization.addCamera(cameraPoses, intrinsics);
    // Write the point cloud mesh and camera mesh to files
    visualization.writeAllMeshes();
}

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
        cv::Mat K_temp = (cv::Mat_<float>(3, 3) << fX, 0.0f, cX,
                        0.0f, fY, cY,
                        0.0f, 0.0f, 1.0f);
        K_temp.convertTo(img->K, CV_64F); // Convert K_temp to double precision and store in img->K
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

        Image *img = findImage(id);
        if (!img) // not found create a new image
        {
            std::cout << "At the point for attaching the extrinsics we need to have created the image with id:" << id << std::endl;
            return false;
        }
        cv::Vec4f q(qx, qy, qz, qw);
        quaternionToRotationMatrix(q, img->R);
        img->t = (cv::Mat_<float>(3, 1) << tx, ty, tz);

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
    for (size_t i = 0; i < 2; ++i) // ADJUST THIS TO LOAD ONLY A FEW IMAGES
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
            cv::Mat depthImage = cv::imread(filenames[i], cv::IMREAD_ANYDEPTH);
            img->depth = depthImage;
            if (!checkLoadImage(depthImage))
                return false;
        }

        img->w = 1.0;
        img->q = 0.0;
    }
    std::cout << "Loaded images: " << images.size() << std::endl;
    return true;
}

void ImageStorage::quaternionToRotationMatrix(const cv::Vec4f &q, cv::Mat &R)
{
    // Ensure the rotation matrix is 3x3
    R = cv::Mat::zeros(3, 3, CV_32F);

    float qw = q[3], qx = q[0], qy = q[1], qz = q[2];

    R.at<float>(0, 0) = 1 - 2 * qy * qy - 2 * qz * qz;
    R.at<float>(0, 1) = 2 * qx * qy - 2 * qz * qw;
    R.at<float>(0, 2) = 2 * qx * qz + 2 * qy * qw;

    R.at<float>(1, 0) = 2 * qx * qy + 2 * qz * qw;
    R.at<float>(1, 1) = 1 - 2 * qx * qx - 2 * qz * qz;
    R.at<float>(1, 2) = 2 * qy * qz - 2 * qx * qw;

    R.at<float>(2, 0) = 2 * qx * qz - 2 * qy * qw;
    R.at<float>(2, 1) = 2 * qy * qz + 2 * qx * qw;
    R.at<float>(2, 2) = 1 - 2 * qx * qx - 2 * qy * qy;
}