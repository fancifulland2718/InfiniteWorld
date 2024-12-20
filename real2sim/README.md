# Real2Sim: Depth-Regulaized PGSR Reconstruction

Ref: 

[PGSR: Planar-based Gaussian Splatting Reconstruction](https://github.com/zju3dv/PGSR)

[3D Gaussian Splatting for Real-Time Radiance Field Rendering](https://github.com/graphdeco-inria/gaussian-splatting/tree/dev?tab=readme-ov-file#depth-regularization)

[Depth Pro: Sharp Monocular Metric Depth in Less Than a Second](https://github.com/apple/ml-depth-pro)  

## Depth-Regulaized PGSR Reconstruction
### Reconstruction Performance Comparison

|           | **GauStudio**          | **SuGaR**             | **PGSR**             | **Ours**              | **GT**                |
|-----------|-------------------------|-----------------------|----------------------|-----------------------|-----------------------|
| **Screen**| ![Screen GS](img/render_result/gs/0112.png) | ![Screen SuGaR](img/render_result/sugar/0112.png) | ![Screen PGSR](img/render_result/pgsr/0112.png) | ![Screen Ours](img/render_result/pgsr_dn/0112.png) | ![Screen GT](img/render_result/gt/0112.png) |
| **Door**  | ![Door GS](img/render_result/gs/0016.png)  | ![Door SuGaR](img/render_result/sugar/0016.png)  | ![Door PGSR](img/render_result/pgsr/0016.png)  | ![Door Ours](img/render_result/pgsr_dn/0016.png)  | ![Door GT](img/render_result/gt/0016.png)  |
| **Wall**  | ![Wall GS](img/render_result/gs/0019.png)  | ![Wall SuGaR](img/render_result/sugar/0019.png)  | ![Wall PGSR](img/render_result/pgsr/0019.png)  | ![Wall Ours](img/render_result/pgsr_dn/0019.png)  | ![Wall GT](img/render_result/gt/0019.png)  |
| **Cabinet**| ![Cabinet GS](img/render_result/gs/0073.png)| ![Cabinet SuGaR](img/render_result/sugar/0073.png)| ![Cabinet PGSR](img/render_result/pgsr/0073.png)| ![Cabinet Ours](img/render_result/pgsr_dn/0073.png)| ![Cabinet GT](img/render_result/gt/0073.png)|
| **Desk**  | ![Desk GS](img/render_result/gs/0024.png)  | ![Desk SuGaR](img/render_result/sugar/0024.png)  | ![Desk PGSR](img/render_result/pgsr/0024.png)  | ![Desk Ours](img/render_result/pgsr_dn/0024.png)  | ![Desk GT](img/render_result/gt/0024.png)  |

**Figure 1:** The reconstruction performance of GauStudio, SuGaR, PGSR, and our proposed method on real-world captured images is evaluated. Compared to 3DGS and SuGaR, PGSR provides an improved visual experience. Building upon PGSR, our method incorporates regularization loss terms for depth and normal vectors, achieving smoother planar surfaces, such as walls, doors, and screens, and demonstrating more robust handling of transparent surfaces like glass.



## Usage

Please refers to README at [post-process](https://github.com/pzhren/InfiniteWorld/blob/master/real2sim/post-process/README.md) and [depth-real2sim](https://github.com/Faccococo/PGSR?tab=readme-ov-file#pgsr-planar-based-gaussian-splatting-for-efficient-and-high-fidelity-surface-reconstruction)
