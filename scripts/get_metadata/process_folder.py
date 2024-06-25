import os
import sys
from subprocess import check_output
from threading import Thread
import json
from tqdm import tqdm

def write_info_for_file(path: str, file: str, pbar: tqdm):
    output = check_output(["./get_metadata", os.path.join(path, file)])
    # catch errors in ./get_metadata
    try:
        json.loads(output)
    except json.decoder.JSONDecodeError:
        print(os.path.join(path, file), output.decode("utf-8"))
        return
    filename = os.path.splitext(file)[0]
    info_path = os.path.join(".", "data", os.path.basename(path), f"{filename}.txt")
    os.makedirs(os.path.join(".", "data"), exist_ok=True)
    os.makedirs(os.path.join(".", "data", os.path.basename(path)), exist_ok=True)
    with open(info_path, "wb") as file:
        file.write(output)
    pbar.update(1)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        raise Exception("Folder path was not provided.")
    path = os.path.normpath(sys.argv[1])
    files = [file for file in os.listdir(sys.argv[1]) if os.path.splitext(file)[1] in [".HEIC", ".jpeg", ".jpg", ".png"]]
    with tqdm(total=len(files)) as pbar:
        threads = set()
        for file in files:
            thread = Thread(target=write_info_for_file, args=(path, file, pbar))
            threads.add(thread)
            thread.start()
        for thread in threads:
            thread.join()