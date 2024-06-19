from pathlib import Path
import argparse
from collections import defaultdict
from tqdm import tqdm
import shutil

ROOT_DIR = Path(__file__).parent.parent


def main():
    # parse the args
    parser = argparse.ArgumentParser(description="Preprocess the freiburg dataset.")
    parser.add_argument(
        "--in_data",
        type=str,
        required=True,
        help="Directory containing the original data",
    )
    parser.add_argument(
        "--out_data",
        type=str,
        required=True,
        help="Directory containing the modified data",
    )
    parser.add_argument(
        "--n",
        type=str,
        required=False,
        help="The number of images to extract.",
        default=None,
    )
    args = parser.parse_args()
    data_dir = ROOT_DIR / "data" / str(args.out_data)
    sfm_data_dir = ROOT_DIR / "data" / str(args.in_data)
    n = args.n

    path = sfm_data_dir / "depth.txt"
    print(f"load {path} ...")
    with open(path, "r") as f:
        lines = f.readlines()
    depth = defaultdict(list)
    idx = 0
    for line in lines:
        if line.startswith("#"):
            continue
        parts = line.split()
        if len(parts) == 2:
            depth["timestamps"].append(float(parts[0]))
            depth["old_filenames"].append(parts[1])
            depth["new_filenames"].append(parts[1].replace(parts[0], f"{idx:05}"))
            depth["id"].append(idx)
            idx += 1
        else:
            raise ValueError

    path = sfm_data_dir / "rgb.txt"
    print(f"load {path} ...")
    with open(path, "r") as f:
        lines = f.readlines()
    rgb = defaultdict(list)
    idx = 0
    for line in lines:
        if line.startswith("#"):
            continue
        parts = line.split()
        if len(parts) == 2:
            rgb["timestamps"].append(float(parts[0]))
            rgb["old_filenames"].append(parts[1])
            rgb["new_filenames"].append(parts[1].replace(parts[0], f"{idx:05}"))
            rgb["id"].append(idx)
            idx += 1
        else:
            raise ValueError

    assert len(rgb["timestamps"]) == len(depth["timestamps"])

    # set the number of the images to extract
    N = len(rgb["timestamps"])
    if n is not None:
        if int(n) > N:
            print(
                f"The value of {n=} is larger then the number of images, set to {N=}."
            )
        else:
            N = int(n)

    # load groundtruth.txt
    path = sfm_data_dir / "groundtruth.txt"
    print(f"load {path} ...")
    with open(path, "r") as f:
        lines = f.readlines()
    groundtruth = defaultdict(list)
    for line in lines:
        if line.startswith("#"):
            continue
        parts = line.split()
        if len(parts) == 8:
            groundtruth["timestamps"].append(float(parts[0]))
            groundtruth["tx"].append(float(parts[1]))
            groundtruth["ty"].append(float(parts[2]))
            groundtruth["tz"].append(float(parts[3]))
            groundtruth["qx"].append(float(parts[4]))
            groundtruth["qy"].append(float(parts[5]))
            groundtruth["qz"].append(float(parts[6]))
            groundtruth["qw"].append(float(parts[7]))
        else:
            raise ValueError

    # load accelerometer.txt
    path = sfm_data_dir / "accelerometer.txt"
    print(f"load {path} ...")
    with open(path, "r") as f:
        lines = f.readlines()
    accelerometer = defaultdict(list)
    for line in lines:
        if line.startswith("#"):
            continue
        parts = line.split()
        if len(parts) == 4:
            accelerometer["timestamps"].append(float(parts[0]))
            accelerometer["ax"].append(float(parts[1]))
            accelerometer["ay"].append(float(parts[2]))
            accelerometer["az"].append(float(parts[3]))
        else:
            raise ValueError

    gt_new = defaultdict(list)
    print("prepare groundruth ...")
    for depth_id in tqdm(range(N), total=N):  # loop over all depth images
        dt = depth["timestamps"][depth_id]
        # find the closest ids for the accelerometer and groundtruth base on depth image timestamps
        delta = max(depth["timestamps"])
        gt_idx = 0
        for gt_id, gtt in enumerate(groundtruth["timestamps"]):
            if abs(dt - gtt) < delta:
                delta = abs(dt - gtt)
                gt_idx = gt_id
        for key, value in groundtruth.items():
            gt_new[key].append(value[gt_idx])
        gt_new["id"].append(depth_id)

    # acc_new = defaultdict(list)
    # print("prepare accelerometer ...")
    # for depth_id in tqdm(range(N), total=N):  # loop over all depth images
    #     dt = depth["timestamps"][depth_id]
    #     # find the closest ids for the accelerometer and groundtruth base on depth image timestamps
    #     delta = max(depth["timestamps"])
    #     gt_idx = 0
    #     for gt_id, gtt in enumerate(accelerometer["timestamps"]):
    #         if abs(dt - gtt) < delta:
    #             delta = abs(dt - gtt)
    #             gt_idx = gt_id
    #     for key, value in accelerometer.items():
    #         acc_new[key].append(value[gt_idx])
    #     acc_new["id"].append(depth_id)

    ###########################################################################
    # Now Save the Files
    ###########################################################################

    print("copy the rgb folder ...")
    data = rgb
    for idx in tqdm(range(N), total=N):
        old_filename = Path(sfm_data_dir / data["old_filenames"][idx])
        new_filename = Path(data_dir / data["new_filenames"][idx])
        new_filename.parent.mkdir(exist_ok=True, parents=True)
        shutil.copy(old_filename, new_filename)

    print("copy the depth folder ...")
    data = depth
    for idx in tqdm(range(N), total=N):
        old_filename = Path(sfm_data_dir / data["old_filenames"][idx])
        new_filename = Path(data_dir / data["new_filenames"][idx])
        new_filename.parent.mkdir(exist_ok=True, parents=True)
        shutil.copy(old_filename, new_filename)

    # create rgb.txt
    path = data_dir / "rgb.txt"
    print(f"save {path} ...")
    data = rgb
    content = "# id filename\n"
    for idx in range(N):
        content += f'{data["id"][idx]:05} {data["new_filenames"][idx]}\n'
    with open(path, "w") as f:
        f.write(content)

    # create depth.txt
    path = data_dir / "depth.txt"
    print(f"save {path} ...")
    data = depth
    content = "# id filename\n"
    for idx in range(N):
        content += f'{data["id"][idx]:05} {data["new_filenames"][idx]}\n'
    with open(path, "w") as f:
        f.write(content)

    # # create accelerometer.txt
    # path = data_dir / "accelerometer.txt"
    # print(f"save {path} ...")
    # data = acc_new
    # content = "# id ax ay az\n"
    # for idx in range(N):
    #     content += f'{data["id"][idx]:05} {data["ax"][idx]} {data["ay"][idx]} {data["az"][idx]}\n'
    # with open(path, "w") as f:
    #     f.write(content)

    # create groundtruth.txt  (extrinsics)
    path = data_dir / "extrinsics.txt"
    print(f"save {path} ...")
    data = gt_new
    content = "# id tx ty tz qx qy qz qw\n"
    for idx in range(N):
        content += f'{data["id"][idx]:05} {data["tx"][idx]} {data["ty"][idx]} {data["tz"][idx]} {data["qx"][idx]} {data["qy"][idx]} {data["qz"][idx]} {data["qw"][idx]}\n'
    with open(path, "w") as f:
        f.write(content)

    path = data_dir / "intrinsics.txt"
    print(f"save {path} ...")
    content = "# id fX fY cX cY\n"
    for idx in range(N):
        content += f"{idx:05} 525.0 525.0 319.5 239.5\n"
    with open(path, "w") as f:
        f.write(content)


if __name__ == "__main__":
    main()
