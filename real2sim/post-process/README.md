# Post-Process

Scripts for converting Stanford Polygon File Format (.ply) meshes to Wavefront Object (.obj) format with additional processing steps.

## Installation

```shell
conda env create -f env.yaml
conda activate 3d
```

## Usage

```shell
./process.sh <path-to-ply>
```

### Processing Steps

1. **Ground Plane Selection**  
    The system identifies the ground plane based on color attributes and mathematical formulas.

2. **Plane Parameters Input**  
    When prompted, enter the ground plane equation coefficients (x, y, z, d).  
    Example: For plane equation `x + 2y + 3z + 1 = 0`, input `1 2 3 1`.

3. **File Generation**  
    The process will generate an `.obj` file in the same directory as the input `.ply` file.

**Note:** Among these steps, only Mesh simplification was used for baseline method comparisons when it does not represent a methodological difference since it was produced after all reconstruction methods. Specially, these processing steps are provided for future simulation purposes.