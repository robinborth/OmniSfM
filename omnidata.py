import torch
import torch.nn.functional as F
from torchvision.transforms import v2

from pathlib import Path
from PIL import Image
import matplotlib.pyplot as plt

from pathlib import Path
import sys

from omnidata_tools.torch.modules.midas.dpt_depth import DPTDepthModel
from omnidata_tools.torch.data.transforms import get_transform

MODEL_DIR = "/Users/robinborth/Code/OmniSfM/models"
DATA_DIR = "/Users/robinborth/Code/OmniSfM/data"


def standardize_depth_map(img, mask_valid=None, trunc_value=0.1):
    if mask_valid is not None:
        img[~mask_valid] = torch.nan
    sorted_img = torch.sort(torch.flatten(img))[0]
    # Remove nan, nan at the end of sort
    num_nan = sorted_img.isnan().sum()
    if num_nan > 0:
        sorted_img = sorted_img[:-num_nan]
    # Remove outliers
    trunc_img = sorted_img[
        int(trunc_value * len(sorted_img)) : int((1 - trunc_value) * len(sorted_img))
    ]
    trunc_mean = trunc_img.mean()
    trunc_var = trunc_img.var()
    eps = 1e-6
    # Replace nan by mean
    img = torch.nan_to_num(img, nan=trunc_mean)
    # Standardize
    img = (img - trunc_mean) / torch.sqrt(trunc_var + eps)
    return img


if __name__ == "__main__":

    task = "depth"
    image_name = "tum_bib.png"
    input_path = Path(DATA_DIR, image_name)
    output_path = Path(DATA_DIR, f"{Path(image_name).stem}_{task}.png")

    # get target task and model
    if task == "normal":
        image_size = 384
        pretrained_weights_path = Path(MODEL_DIR, "omnidata_dpt_normal_v2.ckpt")
        model = DPTDepthModel(backbone="vitb_rn50_384", num_channels=3)
        checkpoint = torch.load(
            pretrained_weights_path, map_location=torch.device("cpu")
        )
        if "state_dict" in checkpoint:
            state_dict = {}
            for k, v in checkpoint["state_dict"].items():
                state_dict[k[6:]] = v
        else:
            state_dict = checkpoint

        model.load_state_dict(state_dict)
        trans_totensor = v2.Compose(
            [
                v2.Resize(image_size),
                v2.CenterCrop(image_size),
                get_transform("rgb", image_size=None),
            ]
        )

    elif task == "depth":
        image_size = 384
        pretrained_weights_path = Path(MODEL_DIR, "omnidata_dpt_depth_v2.ckpt")
        # model = DPTDepthModel(backbone='vitl16_384') # DPT Large
        model = DPTDepthModel(backbone="vitb_rn50_384")  # DPT Hybrid
        checkpoint = torch.load(
            pretrained_weights_path, map_location=torch.device("cpu")
        )
        if "state_dict" in checkpoint:
            state_dict = {}
            for k, v in checkpoint["state_dict"].items():
                state_dict[k[6:]] = v
        else:
            state_dict = checkpoint
        model.load_state_dict(state_dict)
        trans_totensor = v2.Compose(
            [
                v2.Resize(image_size),
                v2.CenterCrop(image_size),
                v2.ToTensor(),
                v2.Normalize(mean=[0.5], std=[0.5]),
            ]
        )
    else:
        print("task should be one of the following: normal, depth")
        sys.exit()

    trans_rgb = v2.Compose([v2.Resize(512), v2.CenterCrop(512)])

    print(f"Reading input {input_path} ...")
    img = Image.open(input_path)
    img_tensor = trans_totensor(img)[:3].unsqueeze(0)
    if img_tensor.shape[1] == 1:
        img_tensor = img_tensor.repeat_interleave(3, 1)

    output = model(img_tensor).clamp(min=0, max=1)

    if task == "depth":
        output = F.interpolate(output.unsqueeze(0), (512, 512), mode="bicubic")
        output = output.squeeze(0).clamp(0, 1)
        output = 1 - output
        # output = standardize_depth_map(output)
        plt.imsave(output_path, output.detach().cpu().squeeze(), cmap="viridis")
    else:
        v2.functional.to_pil_image(output[0]).save(output_path)

    print(f"Writing output {output_path} ...")
