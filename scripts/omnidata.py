import torch
import torch.nn.functional as F
from torchvision.transforms import v2
import matplotlib.pyplot as plt
import sys

from tqdm import tqdm
from pathlib import Path
from PIL import Image
from src.omnidata.dpt_depth import DPTDepthModel
import argparse
import warnings

ROOT_DIR = Path(__file__).parent.parent


def load_model(
    path: str | Path,
    task: str,
    device: str = "cpu",
) -> DPTDepthModel:
    warnings.filterwarnings("ignore", category=UserWarning, module="timm")
    assert task in ["depth", "normal"]
    if task == "depth":
        model = DPTDepthModel(backbone="vitb_rn50_384", num_channels=1)
    else:
        model = DPTDepthModel(backbone="vitb_rn50_384", num_channels=3)
    checkpoint = torch.load(path, map_location=device)
    if "state_dict" in checkpoint:
        state_dict = {}
        for k, v in checkpoint["state_dict"].items():
            state_dict[k[6:]] = v
    else:
        state_dict = checkpoint
    model.load_state_dict(state_dict)
    setattr(model, "task", task)
    return model


def load_image(path: str | Path, task: str):
    # define the transformation for the model
    # image_size = 384
    transforms = [
        # v2.Resize(image_size),
        # v2.CenterCrop(image_size),
        v2.ToImage(),
        v2.ToDtype(torch.float32, scale=True),
    ]
    if task == "depth":
        transforms += [v2.Normalize(mean=[0.5], std=[0.5])]
    transform = v2.Compose(transforms)

    # read the image and transform it
    img = Image.open(path)
    img_tensor = transform(img)[:3].unsqueeze(0)
    if img_tensor.shape[1] == 1:
        img_tensor = img_tensor.repeat_interleave(3, 1)
    return img_tensor


if __name__ == "__main__":
    # parse the args
    parser = argparse.ArgumentParser(description="Process the input data for SfM.")
    parser.add_argument(
        "--data",
        type=str,
        required=True,
        help="Directory containing the data",
    )
    parser.add_argument(
        "--task",
        type=str,
        required=True,
        choices=["depth", "normal"],
        help="Type of processing to perform",
    )
    args = parser.parse_args()
    data_dir = ROOT_DIR / "data" / args.data
    model_dir = ROOT_DIR / "models"
    task = args.task

    # checks for the input
    if task not in ["normal", "depth"]:
        print("ERROR: The task needs to be either normal or depth.")
        sys.exit()

    # get target task and model
    if task == "normal":
        pretrained_weights_path = Path(model_dir, "omnidata_dpt_normal_v2.ckpt")
        model = load_model(pretrained_weights_path, task="normal")
    elif task == "depth":
        pretrained_weights_path = Path(model_dir, "omnidata_dpt_depth_v2.ckpt")
        model = load_model(pretrained_weights_path, task="depth")

    print(f"Preprocess image for task: {task}")
    paths = list(Path(data_dir, "rgb").iterdir())
    for rgb_input in tqdm(paths):
        img_tensor = load_image(rgb_input, task=task)
        output = model(img_tensor).squeeze(0)
        output_path = Path(data_dir, task, rgb_input.name)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        if task == "depth":
            output = output.clamp(0, 1)
            output = 1 - output
            plt.imsave(output_path, output.detach().cpu().squeeze(), cmap="viridis")
            # v2.functional.to_pil_image(output).save(output_path)
        else:
            output = output.clamp(0, 1)
            v2.functional.to_pil_image(output).save(output_path)
