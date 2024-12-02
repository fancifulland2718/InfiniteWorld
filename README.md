# InfiniteWorld

This repo contains code for the paper:

[Arxiv]()

### InfiniteWorld: A Unified Scalable Simulation Framework for General Visual-Language Robot Interaction

[TOC]

![20241202_215955](https://gitee.com/pzhren/img/raw/master/img/202412022200214.png)

### Abstract

Realizing scaling laws in embodied AI has become a focus. However, previous work has been scattered across diverse simulation platforms, with assets and models lacking unified interfaces, which has led to inefficiencies in research. To address this, we introduce InfiniteWorld, a unified and scalable simulator for general vision-language robot interaction built on Nvidia Isaac Sim. InfiniteWorld encompasses a comprehensive set of physics asset construction methods and generalized free robot interaction benchmarks. Specifically, we first built a unified and scalable simulation framework for embodied learning that integrates a series of improvements in generation-driven 3D asset construction, Real2Sim, automated annotation framework, and unified 3D asset processing. This framework provides a unified and scalable platform for robot interaction and learning. In addition, to simulate realistic robot interaction, we build four new general benchmarks, including scene graph collaborative exploration and open-world social mobile manipulation. The former is often overlooked as an important task for robots to explore the environment and build scene knowledge, while the latter simulates robot interaction tasks with different levels of knowledge agents based on the former. They can more comprehensively evaluate the embodied agent's capabilities in environmental understanding, task planning and execution, and intelligent interaction. We hope that this work can provide the community with a systematic asset interface, alleviate the dilemma of the lack of high-quality assets, and provide a more comprehensive evaluation of robot interactions.

### Simulator
![20241202_221116](https://gitee.com/pzhren/img/raw/master/img/202412022211180.png)

Overview of the functions of InfiniteWorld simulator.  Our simulation platform supports different sensors, robot platforms, and teleoperation. In addition, it also realizes unlimited expansion of scene and object assets through generative and Sim2Real methods, and we have also built an annotation platform to reduce annotation costs and improve annotation quality.
#### Language-Driven Automatic Scene Generation and Editing
![20241202_221141](https://gitee.com/pzhren/img/raw/master/img/202412022211858.png)Language-driven automatic scene generation and editing framework based on HOLODECK [77]. It can easily generate various interactive high-fidelity scenes that meet the requirements of users, including scene style replacement, object editing (e.g., adding/removing a  specific number of objects), and replacement (that is, replacing similar objects), etc.
#### Depth-Prior-Constrained Real2Sim

#### Annot8-3D

#### Unified 3D Asset

### Benchmark

![20241202_220957](https://gitee.com/pzhren/img/raw/master/img/202412022210150.png)

### Citation

If you find this code useful in your work, please consider citing

```shell

```