preprocess_freiburg1_xyz:
	python scripts/omnidata.py --data="rgbd_dataset_freiburg1_xyz" --task="depth"
	python scripts/omnidata.py --data="rgbd_dataset_freiburg1_xyz" --task="normal" 