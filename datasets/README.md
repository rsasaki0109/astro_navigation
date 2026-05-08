# Datasets

Downloaded data is not committed. Use:

```bash
python3 ../scripts/download_dataset.py --list
```

Each downloaded dataset is organized as:

```text
datasets/<dataset-key>/
  raw/
  extracted/
  manifest.json
```

Keep public source URLs, checksums, and citations in `manifest.json` so benchmark runs can be
reproduced.

