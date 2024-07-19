# OmniSfM
The SfM project for the lecture 3D Scanning and Motion Capture.

## C++ dependencies using brew

One way of installing c++ dependencies is via [homebrew](https://brew.sh/):

```bash
brew install cmake flann eigen opencv freeimage
```


## Python Setup

To use the omnidata model we use python please ensure that you setup the correct env, in conda it would look like that:

```bash
conda create -n sfm python=3.10        
conda activate sfm
pip install -e .
```

Note that we use pyproject.toml to specify our dependencies.



# Data Structure
The data structure should look like that, note that in order to do inference for the omnidata model or the keypoint detection script we need to have the images in the `/rgb` folder.
To create this run following commands:
```
    cd scripts
    python dataset.py --in_data /path/to/original/rgbd_dataset_freiburg1_xyz --out_data /name/an/output/folder --n number

```


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
cmake .. -DCMAKE_CXX_STANDARD=14
cmake --build .
./sfm
```

