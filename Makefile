preprocess_freiburg1_xyz:
	# python scripts/dataset.py --in_data="rgbd_dataset_freiburg1_xyz" --out_data="freiburg_debug" --n=2
	# python scripts/omnidata.py --data="freiburg_debug" --task="depth" 
	# python scripts/dataset.py --in_data="rgbd_dataset_freiburg1_xyz" --out_data="freiburg_small" --n=10 
	# python scripts/dataset.py --in_data="rgbd_dataset_freiburg1_xyz" --out_data="freiburg_full"
	# python scripts/omnidata.py --data="freiburg_full" --task="depth" 
	# python scripts/dataset.py --in_data="rgbd_dataset_freiburg1_xyz" --out_data="freiburg_large" --n=11
	# python scripts/omnidata.py --data="freiburg_large" --task="depth" 
	python scripts/dataset.py --in_data="rgbd_dataset_freiburg1_xyz" --out_data="freiburg_3" --n=51

get_freiburg1_xyz:
	wget https://vision.in.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_xyz.tgz
	tar -xvzf rgbd_dataset_freiburg1_xyz.tgz

preprocess_demo:
	python scripts/omnidata.py --data="demo" --task="depth" 
	python scripts/omnidata.py --data="demo" --task="normal" 
