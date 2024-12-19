<div align="center">
<h1> :earth_africa: InfiniteWorld </h1>
<h3>A Unified Scalable Simulation Framework for General Visual-Language Robot Interaction</h3>
    
Pengzhen Ren*, Min Li*, Zhen Luo*, Xinshuai Song*, Ziwei Chen*, Weijia Liufu*, Yixuan Yang*, Hao Zheng*

Rongtao Xu, Zitong Huang, Tongsheng Ding, Luyang Xie, Kaidong Zhang, Changfei Fu, Yang Liu, Liang Lin, Feng Zheng<sup>:email:</sup>, Xiaodan Liang<sup>:email:</sup>

<sup>* </sup>equal contribution.   <sup>:email:</sup> corresponding author.

[[`Paper`](https://arxiv.org/abs/2412.05789).]
</div>

![20241202_215955](https://gitee.com/pzhren/img/raw/master/img/202412022200214.png)

## :rocket: Introduction

* We have built a unified and scalable simulation framework that integrates various improved and latest embodied asset reconstruction methods. This has greatly alleviated the community's plight of lacking high-quality embodied assets.
* We build a complete web-based smart point cloud automatic annotation framework that supports distributed collaboration, AI assistance, and optional human-in-the-loop features. This provides strong support for complex robot interactions.
* We designed systematic benchmarks for robot interaction, including scene graph collaborative exploration and open-world social mobile manipulation. This provides a comprehensive and systematic evaluation of the capabilities of embodied agents in perception, planning, execution, and communication.

## :page_facing_up: Simulator
![20241202_221116](https://gitee.com/pzhren/img/raw/master/img/202412022211180.png)

Overview of the functions of InfiniteWorld simulator.  Our simulation platform supports different sensors, robot platforms, and teleoperation. In addition, it also realizes unlimited expansion of scene and object assets through generative and Sim2Real methods, and we have also built an annotation platform to reduce annotation costs and improve annotation quality.
### Language-Driven Automatic Scene Generation and Editing
![20241202_221141](https://gitee.com/pzhren/img/raw/master/img/202412022211858.png)Language-driven automatic scene generation and editing framework based on HOLODECK [77]. It can easily generate various interactive high-fidelity scenes that meet the requirements of users, including scene style replacement, object editing (e.g., adding/removing a  specific number of objects), and replacement (that is, replacing similar objects), etc.
### Depth-Prior-Constrained Real2Sim
We use the depth prior to constrain the generation of the 3D model, which enhances simulation realism by integrating depth priors and normal priors into the Planar-based Gaussian Splatting Reconstruction (PGSR) framework.

Additionally, this project contains Python scripts for post-processing reconstructed mesh scene `ply` files, which could help you optimize, repair, and enhance the 3D mesh. Especially, an example of a conda environment `real2sim/post-process/env.yaml` is provided, which has been tested to successfully run all scripts on Windows 11. 

### Annot8-3D

The 3D annotation framework is open-sourced at: https://github.com/zh-plus/annot8-3d.

### Unified 3D Asset

We provide code for converting 3D datasets into different formats in `3D format convert`. Using `ply2obj_replica.py`, you can convert Replica data from PLY format to OBJ format, but it requires installing the `pymeshlab` library. The script `obj_usd.py` allows you to convert objects in OBJ format to USD format for use in Isaac Sim. For batch conversions, you can use `obj_usd0.py`, though its compatibility with Isaac Sim is limited. Additionally, `pkl_obj.py` converts files from PKL to OBJ format, which is the first step in adapting the Objaverse dataset for use in Isaac Sim.

## Benchmark

![20241202_220957](https://gitee.com/pzhren/img/raw/master/img/202412022210150.png)

## Citation

If you find this code useful in your work, please consider citing

```shell
@misc{ren2024infiniteworld,
    title={InfiniteWorld: A Unified Scalable Simulation Framework for General Visual-Language Robot Interaction},
    author={Pengzhen Ren and Min Li and Zhen Luo and Xinshuai Song and Ziwei Chen and Weijia Liufu and Yixuan Yang and Hao Zheng and Rongtao Xu and Zitong Huang and Tongsheng Ding and Luyang Xie and Kaidong Zhang and Changfei Fu and Yang Liu and Liang Lin and Feng Zheng and Xiaodan Liang},
    year={2024},
    eprint={2412.05789},
    archivePrefix={arXiv},
    primaryClass={cs.RO}
}
