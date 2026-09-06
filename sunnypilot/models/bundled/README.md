# Bundled Pop Model v2

Shipped so a fresh C3XL install can start `modeld_tinygrad` without waiting
for ModelManager to download from GitLab. Chunks and hashes match
`driving_models_v18.json` (PMV2, index 59).

Do not re-chunk these files. `common/file_chunker.py` and the catalog both
use 45 MiB pieces.
