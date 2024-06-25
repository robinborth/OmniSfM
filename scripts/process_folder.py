from typing import Tuple
from pillow_heif import register_heif_opener
from PIL import Image
from argparse import ArgumentParser
import os
from tqdm import tqdm
from threading import Thread
import json
from subprocess import check_output

register_heif_opener()

def convert_to_png(in_path: str, out_path: str, pbar: tqdm, file, index: int):
  img = Image.open(in_path)
  img.save(out_path)
  exif_data = check_output(["./get_metadata/get_metadata", in_path])
  fX, fY, cX, cY = get_intrinsics(exif_data)
  file.write(f"{index:05d} {fX} {fY} {cX} {cY}\n")
  pbar.update(1)

def main():
  parser = ArgumentParser()
  parser.add_argument(
    "--in_data",
    type=str,
    required=True
  )
  parser.add_argument(
    "--out_data",
    type=str,
    required=True
  )
  args = parser.parse_args()
  
  in_data = os.path.normpath(args.in_data)
  out_data = os.path.normpath(args.out_data)

  files = [file for file in os.listdir(in_data) if os.path.splitext(file)[1] in [".HEIC", ".jpeg", ".jpg", ".png"]]

  rgb_dir = os.path.join(out_data, "rgb")
  os.makedirs(rgb_dir, exist_ok=True)

  with open(os.path.join(rgb_dir, "intrinsics.txt"), "w") as intrinsics_file:
    intrinsics_file.write("# id fX fY cX cY\n")
    with tqdm(total=len(files), desc="Copy and convert images") as pbar:
      threads = set()
      # copy and convert images
      for index, file in enumerate(files):
        img_path = os.path.join(rgb_dir, f"{index:05d}.png")
        thread = Thread(target=convert_to_png, args=(os.path.join(in_data, file), img_path, pbar, intrinsics_file, index))
        threads.add(thread)
        thread.start()
      for thread in threads:
        thread.join()

def get_intrinsics(exif_str: str) -> Tuple[float, float, float, float]:
  exif = json.loads(exif_str)
  focal_length = exif["FocalLength"]
  width = exif["PixelXDimension"]
  height = exif["PixelYDimension"]
  pixel_size_x = 1 / width
  pixel_size_y = 1 / height
  fX = focal_length / pixel_size_x
  fY = focal_length / pixel_size_y
  cX = width / 2
  cY = height / 2
  return (fX, fY, cX, cY)

if __name__ == "__main__":
  main()