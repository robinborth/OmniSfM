preprocess_freiburg1_xyz:
	python scripts/dataset.py --in_data="rgbd_dataset_freiburg1_xyz" --out_data="freiburg_small" --n=10 
	python scripts/dataset.py --in_data="rgbd_dataset_freiburg1_xyz" --out_data="freiburg_full"
	python scripts/omnidata.py --data="freiburg_small" --task="depth" 
	python scripts/omnidata.py --data="freiburg_full" --task="depth" 

preprocess_demo:
	python scripts/omnidata.py --data="demo" --task="depth" 
	python scripts/omnidata.py --data="demo" --task="normal" 
