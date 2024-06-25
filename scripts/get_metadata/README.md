# Usage

## File

```bash
./get_metadata <path/to/image>
```

Prints a JSON string of exif metadata to console.

## Folder

```bash
python process_folder.py <path/to/folder>
```

For every file in the folder, a `.txt` file is created at `/data/<foldername>`, containg the exif data as a JSON string.