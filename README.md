# OmniSfM
The SfM project for the lecture 3D Scanning and Motion Capture.


# Install Omnidata

Go to https://github.com/EPFL-VILAB/omnidata.git and clone into the /libs folder.
Follow: https://github.com/EPFL-VILAB/omnidata/tree/main/omnidata_tools/torch#pretrained-models to install the library.
Make sure taht you are in the omnidata/omnidata_tools/torch folder and then run:
```bash
conda create -n sfm -y python=3.8
source activate sfm
pip install -r requirements.txt
```

## Download pretrained models

```bash
https://zenodo.org/records/10447888/files/omnidata_dpt_depth_v2.ckpt?download=1
https://zenodo.org/records/10447888/files/omnidata_dpt_normal_v2.ckpt?download=1
```