# OmniSfM
The SfM project for the lecture 3D Scanning and Motion Capture.

## C++ dependencies using brew

One way of installing c++ dependencies is via [homebrew](https://brew.sh/):

```bash
brew install cmake flann eigen opencv freeimage
```

# Omnidata Model

## Python Setup

To use the omnidata model we use python please ensure that you setup the correct env, in conda it would look like that:

```bash
conda create -n sfm python=3.10        
conda activate sfm
pip install -e .
```

Note that we use pyproject.toml to specify our dependencies.

## Download Omnidata Models

In order to do inference with the omnidata models please download them via the following links:

```bash
https://zenodo.org/records/10447888/files/omnidata_dpt_depth_v2.ckpt?download=1
https://zenodo.org/records/10447888/files/omnidata_dpt_normal_v2.ckpt?download=1
```

After you downloaded them put them into the models folder like that:

```bash
models
├── omnidata_dpt_depth_v2.ckpt
└── omnidata_dpt_normal_v2.ckpt
```

## Inference

In order to do inference you can just run, see Makefile which options you have.

```bash
python scripts/omnidata.py --data="rgbd_dataset_freiburg1_xyz" --task="depth"
```

# Data Structure
The data structure should look like that, note that in order to do inference for the omnidata model or the keypoint detection skript we need to have the images in the `/rgb` folder.

```bash
data
└── rgbd_dataset_freiburg1_xyz
    ├── depth
    │   ├── 00000.png
    │   ├── 00001.png
    │   └── 00002.png
    ├── normal
    │   ├── 00000.png
    │   ├── 00001.png
    │   └── 00002.png
    └── rgb
        ├── 00000.png
        ├── 00001.png
        └── 00002.png
```


## OpenCV

We use the version opencv-4.9.0 please clone the repo and build it in your libs folder.
We need to make sure that the cpp version is 14, hence do the following:


```bash
mkdir build && cd build 
cmake .. -DCMAKE_CXX_STANDARD=14 -DCMAKE_INSTALL_PREFIX=/Users/robinborth/Code/OmniSfM/libs/opencv
cmake --build . --target install
```

## Open3D

We use the version open3d-0.18.0 please clone the repo and build it in your libs folder.


# TODOS
- Preprocess Images From iPhone, e.g. HEIC format, extract intrinsics.
- Visualizer for the PointCloud 
- Dense PointCloud -> Mesh 
- Approximate nearest neighbour serach, with FLANN?
- NERF-Datset with GT reconstructions, preperation for abblations.

