# Post-Process

Scripts for post-processing the Stanford Polygon File Format(.ply) mesh and transform to Wavefront Object(.obj) format.

## Installation：

```shell
conda env create -f env.yaml
conda activate 3d
```

## Usage

```shell
./process.sh <path-to-ply>
```

### Step 1. 
Based on the plane color and the corresponding formula, select the ground plane.  
### Step 2. 
In the upcoming command line, input the ground plane formula's x, y, z, d values as prompted (e.g., if the ground plane formula is `x + 2y + 3z + 1 = 0`, input `1 2 3 1` in sequence).  
### Step 3. 
Wait for the process to complete. Once done, an `obj` file will be generated in the `ply` directory.  
