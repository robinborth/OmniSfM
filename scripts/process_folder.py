from pillow_heif import register_heif_opener
from PIL import Image
from argparse import ArgumentParser
import os
from tqdm import tqdm
from threading import Thread

register_heif_opener()

def convert_to_png(in_path: str, out_path: str, pbar: tqdm):
  img = Image.open(in_path)
  img.save(out_path)
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

  with tqdm(total=len(files), desc="Copy and convert images") as pbar:
    threads = set()
    # copy and convert images
    for index, file in enumerate(files):
      img_path = os.path.join(rgb_dir, f"{index:05d}.png")
      thread = Thread(target=convert_to_png, args=(os.path.join(in_data, file), img_path, pbar))
      threads.add(thread)
      thread.start()
    for thread in threads:
      thread.join()

if __name__ == "__main__":
  main()