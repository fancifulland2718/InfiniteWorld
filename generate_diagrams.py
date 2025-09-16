#!/usr/bin/env python3
"""
InfiniteWorld Architecture Diagram Generator
生成 InfiniteWorld 仿真平台的架构图
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import FancyBboxPatch
import numpy as np

def create_architecture_diagram():
    """创建 InfiniteWorld 架构图"""
    
    # 设置中文字体
    plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans', 'Arial Unicode MS']
    plt.rcParams['axes.unicode_minus'] = False
    
    # 创建图形
    fig, ax = plt.subplots(1, 1, figsize=(16, 12))
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 10)
    ax.axis('off')
    
    # 定义颜色
    colors = {
        'user': '#E3F2FD',      # 浅蓝色 - 用户层
        'app': '#F3E5F5',       # 浅紫色 - 应用层  
        'robot': '#E8F5E8',     # 浅绿色 - 机器人层
        'core': '#FFF3E0',      # 浅橙色 - 核心层
        'scene': '#F1F8E9',     # 浅青色 - 场景层
        'data': '#E0F2F1',      # 浅薄荷色 - 数据层
        'infra': '#FAFAFA'      # 浅灰色 - 基础设施层
    }
    
    # 绘制各层级
    layers = [
        # (y_start, height, color, title, components)
        (9, 0.8, colors['user'], '用户交互层 (User Interface Layer)', 
         ['语言指令', '键盘控制', 'Web标注', '可视化', 'API接口']),
        
        (8, 0.8, colors['app'], '应用层 (Application Layer)',
         ['基准测试1', '基准测试2', '键盘控制', '导航操作']),
        
        (6.8, 1, colors['robot'], '机器人控制层 (Robot Control Layer)',
         ['Stretch移动机器人', 'Unitree H1人形机器人']),
        
        (5.5, 1, colors['core'], '仿真核心层 (Simulation Core Layer)',
         ['NVIDIA Isaac Sim 4.0.0', 'Omniverse Platform']),
        
        (4.2, 1, colors['scene'], '场景和资产层 (Scene and Asset Layer)',
         ['智能场景生成', 'Real2Sim重建', '3D资产管理']),
        
        (2.9, 1, colors['data'], '数据和算法层 (Data and Algorithm Layer)',
         ['路径规划', '视觉感知', '语义理解', '学习算法']),
        
        (1.6, 1, colors['infra'], '基础设施层 (Infrastructure Layer)',
         ['PyTorch', 'Open3D', 'Trimesh', 'OpenCV']),
    ]
    
    # 绘制每一层
    for y_start, height, color, title, components in layers:
        # 绘制层级背景
        rect = FancyBboxPatch((0.2, y_start-height), 9.6, height,
                             boxstyle="round,pad=0.02", 
                             facecolor=color, 
                             edgecolor='gray',
                             linewidth=1)
        ax.add_patch(rect)
        
        # 添加层级标题
        ax.text(5, y_start-0.15, title, ha='center', va='center', 
                fontsize=12, fontweight='bold')
        
        # 添加组件
        if len(components) <= 4:
            x_positions = np.linspace(1, 9, len(components))
        else:
            x_positions = np.linspace(0.8, 9.2, len(components))
            
        for i, comp in enumerate(components):
            ax.text(x_positions[i], y_start-0.5, comp, ha='center', va='center',
                   fontsize=9, bbox=dict(boxstyle="round,pad=0.3", 
                   facecolor='white', alpha=0.8))
    
    # 绘制连接箭头
    arrow_props = dict(arrowstyle='->', connectionstyle='arc3', 
                      color='darkblue', lw=2)
    
    # 层间连接
    for i in range(len(layers)-1):
        y_from = layers[i][0] - layers[i][1]
        y_to = layers[i+1][0]
        ax.annotate('', xy=(5, y_to), xytext=(5, y_from),
                   arrowprops=arrow_props)
    
    # 添加侧边说明框
    side_box = FancyBboxPatch((10.2, 3), 3.5, 4,
                             boxstyle="round,pad=0.1", 
                             facecolor='#F5F5F5', 
                             edgecolor='gray',
                             linewidth=1)
    ax.add_patch(side_box)
    
    # 侧边说明文字
    side_text = [
        "关键特性:",
        "• 统一可扩展架构",
        "• 深度先验Real2Sim", 
        "• 语言驱动场景生成",
        "• 多机器人平台支持",
        "• 全面基准测试体系",
        "• 大规模3D资产库",
        "• 模块化开发工具"
    ]
    
    for i, text in enumerate(side_text):
        weight = 'bold' if i == 0 else 'normal'
        size = 11 if i == 0 else 9
        ax.text(10.4, 6.7-i*0.4, text, ha='left', va='center',
               fontsize=size, fontweight=weight)
    
    # 添加主标题
    ax.text(5, 9.7, 'InfiniteWorld 仿真平台架构图', ha='center', va='center',
           fontsize=16, fontweight='bold')
    
    # 调整布局和保存
    plt.subplots_adjust(left=0.05, right=0.95, top=0.95, bottom=0.05)
    plt.savefig('/home/runner/work/InfiniteWorld/InfiniteWorld/architecture_diagram.png', 
                dpi=300, bbox_inches='tight', facecolor='white')
    plt.savefig('/home/runner/work/InfiniteWorld/InfiniteWorld/architecture_diagram.pdf', 
                bbox_inches='tight', facecolor='white')
    
    print("架构图已生成:")
    print("- architecture_diagram.png")  
    print("- architecture_diagram.pdf")
    
    return fig

def create_data_flow_diagram():
    """创建数据流图"""
    
    fig, ax = plt.subplots(1, 1, figsize=(14, 8))
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 6)
    ax.axis('off')
    
    # 数据流步骤
    steps = [
        (1, 3, "外部输入\nExternal Input"),
        (2.5, 3, "预处理\nPreprocessing"), 
        (4, 3, "仿真环境\nSimulation Env"),
        (5.5, 3, "机器人控制\nRobot Control"),
        (7, 3, "执行反馈\nExecution Feedback"),
        (8.5, 3, "结果输出\nOutput Results")
    ]
    
    # 绘制步骤框
    for x, y, text in steps:
        rect = FancyBboxPatch((x-0.4, y-0.4), 0.8, 0.8,
                             boxstyle="round,pad=0.05",
                             facecolor='lightblue',
                             edgecolor='darkblue')
        ax.add_patch(rect)
        ax.text(x, y, text, ha='center', va='center', fontsize=9, fontweight='bold')
    
    # 绘制箭头
    arrow_props = dict(arrowstyle='->', lw=2, color='darkblue')
    for i in range(len(steps)-1):
        x1, y1, _ = steps[i]
        x2, y2, _ = steps[i+1]
        ax.annotate('', xy=(x2-0.4, y2), xytext=(x1+0.4, y1), 
                   arrowprops=arrow_props)
    
    # 添加详细说明
    details = [
        (1, 2, "• 语言指令\n• 3D场景\n• 实时控制"),
        (2.5, 2, "• 格式转换\n• 质量控制\n• 网格优化"),
        (4, 2, "• 场景加载\n• 物理仿真\n• 渲染显示"),
        (5.5, 2, "• 策略推理\n• 传感融合\n• 路径规划"),
        (7, 2, "• 动作执行\n• 状态更新\n• 碰撞检测"),
        (8.5, 2, "• 性能评估\n• 数据记录\n• 可视化")
    ]
    
    for x, y, text in details:
        ax.text(x, y, text, ha='center', va='top', fontsize=8,
               bbox=dict(boxstyle="round,pad=0.2", facecolor='lightyellow', alpha=0.8))
    
    # 添加标题
    ax.text(5, 5, 'InfiniteWorld 数据流程图', ha='center', va='center',
           fontsize=14, fontweight='bold')
    
    plt.savefig('/home/runner/work/InfiniteWorld/InfiniteWorld/data_flow_diagram.png', 
                dpi=300, bbox_inches='tight', facecolor='white')
    print("数据流图已生成: data_flow_diagram.png")
    
    return fig

if __name__ == "__main__":
    print("正在生成 InfiniteWorld 架构图...")
    
    # 生成架构图
    arch_fig = create_architecture_diagram()
    
    # 生成数据流图  
    flow_fig = create_data_flow_diagram()
    
    print("\n图表生成完成!")
    print("请查看生成的文件:")
    print("1. architecture_diagram.png - 主架构图")
    print("2. architecture_diagram.pdf - 主架构图(PDF)")
    print("3. data_flow_diagram.png - 数据流图")