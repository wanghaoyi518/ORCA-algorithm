#!/usr/bin/env python3
"""
ORCA算法多智能体导航可视化工具
用于为ORCA算法的single_test和series_test生成可视化GIF动画
"""

import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle, Polygon
import numpy as np
import argparse
import os
import sys
from pathlib import Path
import seaborn as sns

class ORCAVisualizer:
    def __init__(self, task_file=None, log_file=None):
        """
        初始化可视化器
        
        Args:
            task_file: 任务XML文件路径
            log_file: 日志XML文件路径（可选）
        """
        self.task_file = task_file
        self.log_file = log_file
        
        # 地图信息
        self.map_width = 0
        self.map_height = 0
        self.cell_size = 1
        self.grid = None
        
        # 智能体信息
        self.agents = {}
        self.agent_paths = {}
        self.agent_radius = 0.3
        
        # 障碍物信息
        self.obstacles = []
        
        # 可视化参数
        self.colors = None
        self.fig = None
        self.ax = None
        self.show_grid = True  # 添加网格显示选项
        
    def parse_task_file(self):
        """解析任务XML文件"""
        if not self.task_file or not os.path.exists(self.task_file):
            raise FileNotFoundError(f"任务文件不存在: {self.task_file}")
        
        tree = ET.parse(self.task_file)
        root = tree.getroot()
        
        # 解析智能体信息
        agents_elem = root.find('agents')
        default_params = agents_elem.find('default_parameters')
        if default_params is not None:
            self.agent_radius = float(default_params.get('size', 0.3))
        
        for agent_elem in agents_elem.findall('agent'):
            agent_id = int(agent_elem.get('id'))
            start_x = float(agent_elem.get('start.xr'))
            start_y = float(agent_elem.get('start.yr'))
            goal_x = float(agent_elem.get('goal.xr'))
            goal_y = float(agent_elem.get('goal.yr'))
            
            self.agents[agent_id] = {
                'start': (start_x, start_y),
                'goal': (goal_x, goal_y)
            }
        
        # 解析地图信息
        map_elem = root.find('map')
        self.map_width = int(map_elem.find('width').text)
        self.map_height = int(map_elem.find('height').text)
        
        cellsize_elem = map_elem.find('cellsize')
        if cellsize_elem is not None:
            self.cell_size = float(cellsize_elem.text)
        
        # 解析网格（如果存在）
        grid_elem = map_elem.find('grid')
        if grid_elem is not None:
            self.grid = []
            for row_elem in grid_elem.findall('row'):
                row = list(map(int, row_elem.text.split()))
                self.grid.append(row)
        
        # 解析障碍物
        obstacles_elem = root.find('obstacles')
        if obstacles_elem is not None:
            for obstacle_elem in obstacles_elem.findall('obstacle'):
                vertices = []
                for vertex_elem in obstacle_elem.findall('vertex'):
                    x = float(vertex_elem.get('xr'))
                    y = float(vertex_elem.get('yr'))
                    vertices.append((x, y))
                self.obstacles.append(vertices)
    
    def parse_log_file(self):
        """解析日志XML文件"""
        if not self.log_file or not os.path.exists(self.log_file):
            print(f"警告: 日志文件不存在: {self.log_file}")
            return
        
        tree = ET.parse(self.log_file)
        root = tree.getroot()
        
        # 查找log元素
        log_elem = root.find('log')
        if log_elem is None:
            print("警告: 日志文件中没有找到log元素")
            return
        
        # 解析智能体路径
        for agent_elem in log_elem.findall('agent'):
            agent_id = int(agent_elem.get('id'))
            path_elem = agent_elem.find('path')
            
            if path_elem is not None:
                path = []
                for step_elem in path_elem.findall('step'):
                    x = float(step_elem.get('xr'))
                    y = float(step_elem.get('yr'))
                    path.append((x, y))
                
                self.agent_paths[agent_id] = path
    
    def setup_visualization(self):
        """设置可视化参数"""
        # 为每个智能体分配颜色
        n_agents = len(self.agents)
        if n_agents > 0:
            self.colors = sns.color_palette("husl", n_agents)
        
        # 创建图形
        self.fig, self.ax = plt.subplots(figsize=(12, 10))
        self.ax.set_xlim(0, self.map_width)
        self.ax.set_ylim(0, self.map_height)
        self.ax.set_aspect('equal')
        self.ax.set_xlabel('X Coordinate')
        self.ax.set_ylabel('Y Coordinate')
        self.ax.set_title('ORCA Multi-Agent Navigation Visualization')
        
        # 绘制网格背景（如果存在）
        if self.grid and self.show_grid:
            grid_array = np.array(self.grid)
            self.ax.imshow(grid_array, extent=[0, self.map_width, 0, self.map_height], 
                          origin='lower', cmap='Greys', alpha=0.3)
        
        # 绘制网格线
        if self.show_grid:
            self._draw_grid_lines()
        
        # 绘制障碍物
        for obstacle in self.obstacles:
            if len(obstacle) > 2:
                polygon = Polygon(obstacle, alpha=0.7, facecolor='gray', edgecolor='black')
                self.ax.add_patch(polygon)
    
    def _draw_grid_lines(self):
        """绘制网格线"""
        # 绘制垂直线
        for x in range(int(self.map_width) + 1):
            self.ax.axvline(x=x, color='lightgray', linestyle='-', alpha=0.5, linewidth=0.5)
        
        # 绘制水平线
        for y in range(int(self.map_height) + 1):
            self.ax.axhline(y=y, color='lightgray', linestyle='-', alpha=0.5, linewidth=0.5)
    
    def create_animation(self, output_file='orca_animation.gif', fps=10, trail_length=20):
        """
        创建动画GIF
        
        Args:
            output_file: 输出文件名
            fps: 帧率
            trail_length: 轨迹长度（显示的历史位置数量）
        """
        self.setup_visualization()
        
        # 如果没有路径数据，只显示起始和目标位置
        if not self.agent_paths:
            self._create_static_visualization(output_file)
            return
        
        # 计算最大步数
        max_steps = max(len(path) for path in self.agent_paths.values()) if self.agent_paths else 1
        
        # 为每个智能体创建圆形和轨迹
        agent_circles = {}
        agent_trails = {}
        agent_goal_markers = {}
        
        for i, (agent_id, agent_info) in enumerate(self.agents.items()):
            color = self.colors[i] if self.colors else 'blue'
            
            # 智能体圆形
            circle = Circle((0, 0), self.agent_radius, color=color, alpha=0.7)
            self.ax.add_patch(circle)
            agent_circles[agent_id] = circle
            
            # 轨迹线
            trail_line, = self.ax.plot([], [], color=color, alpha=0.3, linewidth=1)
            agent_trails[agent_id] = trail_line
            
            # 起始位置标记（方形）
            start_x, start_y = agent_info['start']
            start_marker = self.ax.scatter(start_x, start_y, marker='s', s=80, 
                                         color=color, edgecolor='black', alpha=0.8,
                                         label=f'Agent {agent_id} Start')
            
            # 目标位置标记
            goal_x, goal_y = agent_info['goal']
            goal_marker = self.ax.scatter(goal_x, goal_y, marker='*', s=100, 
                                        color=color, edgecolor='black', alpha=0.8,
                                        label=f'Agent {agent_id} Goal')
            agent_goal_markers[agent_id] = goal_marker
        
        # 添加图例
        self.ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        
        def animate(frame):
            """动画更新函数"""
            for agent_id in self.agents:
                if agent_id in self.agent_paths and frame < len(self.agent_paths[agent_id]):
                    # 获取当前位置
                    x, y = self.agent_paths[agent_id][frame]
                    
                    # 更新智能体位置
                    agent_circles[agent_id].center = (x, y)
                    
                    # 更新轨迹
                    start_idx = max(0, frame - trail_length)
                    trail_points = self.agent_paths[agent_id][start_idx:frame+1]
                    if len(trail_points) > 1:
                        trail_x, trail_y = zip(*trail_points)
                        agent_trails[agent_id].set_data(trail_x, trail_y)
                    
                elif agent_id in self.agents:
                    # 如果没有路径数据，显示起始位置
                    start_x, start_y = self.agents[agent_id]['start']
                    agent_circles[agent_id].center = (start_x, start_y)
            
            return list(agent_circles.values()) + list(agent_trails.values())
        
        # 创建动画
        anim = animation.FuncAnimation(self.fig, animate, frames=max_steps,
                                     interval=1000//fps, blit=True, repeat=True)
        
        # 保存动画
        print(f"正在生成动画: {output_file}")
        anim.save(output_file, writer='pillow', fps=fps)
        print(f"动画已保存: {output_file}")
        
        plt.tight_layout()
        return self.fig
    
    def _create_static_visualization(self, output_file):
        """创建静态可视化（当没有路径数据时）"""
        print("没有路径数据，创建静态可视化图...")
        
        for i, (agent_id, agent_info) in enumerate(self.agents.items()):
            color = self.colors[i] if self.colors else 'blue'
            
            # 起始位置（方形标记）
            start_x, start_y = agent_info['start']
            self.ax.scatter(start_x, start_y, marker='s', s=80, color=color, 
                          edgecolor='black', alpha=0.8, label=f'Agent {agent_id} Start')
            
            # 在起始位置添加agent ID标识
            circle = Circle((start_x, start_y), self.agent_radius, color=color, alpha=0.7)
            self.ax.add_patch(circle)
            self.ax.text(start_x, start_y, str(agent_id), ha='center', va='center', 
                        fontsize=8, fontweight='bold')
            
            # 目标位置
            goal_x, goal_y = agent_info['goal']
            self.ax.scatter(goal_x, goal_y, marker='*', s=100, color=color, 
                          edgecolor='black', alpha=0.8, label=f'Agent {agent_id}')
            
            # 连接线
            self.ax.plot([start_x, goal_x], [start_y, goal_y], 
                        color=color, alpha=0.3, linestyle='--')
        
        self.ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        plt.tight_layout()
        
        # 保存静态图片
        static_file = output_file.replace('.gif', '_static.png')
        plt.savefig(static_file, dpi=150, bbox_inches='tight')
        print(f"静态图片已保存: {static_file}")


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='ORCA算法可视化工具')
    parser.add_argument('task_file', help='任务XML文件路径')
    parser.add_argument('--log_file', help='日志XML文件路径（可选）')
    parser.add_argument('--output', '-o', default='orca_animation.gif', 
                       help='输出GIF文件名（默认: orca_animation.gif）')
    parser.add_argument('--fps', type=int, default=10, help='动画帧率（默认: 10）')
    parser.add_argument('--trail', type=int, default=20, 
                       help='轨迹长度，即显示的历史位置数量（默认: 20）')
    parser.add_argument('--grid', action='store_true', help='显示网格背景')
    
    args = parser.parse_args()
    
    # 检查输入文件
    if not os.path.exists(args.task_file):
        print(f"错误: 任务文件不存在: {args.task_file}")
        sys.exit(1)
    
    if args.log_file and not os.path.exists(args.log_file):
        print(f"警告: 日志文件不存在: {args.log_file}")
    
    # 创建可视化器
    visualizer = ORCAVisualizer(args.task_file, args.log_file)
    visualizer.show_grid = args.grid # 根据命令行参数设置网格显示
    
    try:
        # 解析文件
        print("解析任务文件...")
        visualizer.parse_task_file()
        print(f"发现 {len(visualizer.agents)} 个智能体")
        
        if args.log_file:
            print("解析日志文件...")
            visualizer.parse_log_file()
            print(f"解析了 {len(visualizer.agent_paths)} 个智能体的路径数据")
        
        # 创建动画
        visualizer.create_animation(args.output, args.fps, args.trail)
        
    except Exception as e:
        print(f"错误: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main() 