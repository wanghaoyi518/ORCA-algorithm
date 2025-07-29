#!/usr/bin/env python3
"""
Test script for improved Push and Rotate implementation
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from python_pnr.push_and_rotate import PushAndRotate
from python_pnr.sub_map import SubMap
from python_pnr.actor import Actor
from python_pnr.actor_set import ActorSet
from python_pnr.mapf_config import MAPFConfig
from python_pnr.node import Point, Node

def create_deadlock_scenario():
    """Create a scenario that typically causes deadlocks"""
    # Create a corridor scenario where agents need to swap positions
    grid = [
        [1, 1, 1, 1, 1, 1, 1],
        [1, 0, 0, 0, 0, 0, 1],
        [1, 0, 0, 0, 0, 0, 1], 
        [1, 0, 0, 0, 0, 0, 1],
        [1, 0, 0, 0, 0, 0, 1],
        [1, 0, 0, 0, 0, 0, 1],
        [1, 1, 1, 1, 1, 1, 1]
    ]
    
    sub_map = SubMap(grid)
    
    # Create actors that need to swap positions
    actors = []
    
    # Actor 0: start at (1,1), goal at (5,5)
    actor0 = Actor(0, Point(1, 1), Point(5, 5))
    actors.append(actor0)
    
    # Actor 1: start at (5,5), goal at (1,1) 
    actor1 = Actor(1, Point(5, 5), Point(1, 1))
    actors.append(actor1)
    
    # Actor 2: start at (1,3), goal at (5,3)
    actor2 = Actor(2, Point(1, 3), Point(5, 3))
    actors.append(actor2)
    
    # Actor 3: start at (5,3), goal at (1,3)
    actor3 = Actor(3, Point(5, 3), Point(1, 3))
    actors.append(actor3)
    
    actor_set = ActorSet()
    for actor in actors:
        actor_set.add_actor(actor)
    
    return sub_map, actor_set

def test_improved_par():
    """Test the improved Push and Rotate implementation"""
    print("Testing improved Push and Rotate implementation...")
    
    # Create test scenario
    sub_map, actor_set = create_deadlock_scenario()
    
    # Create configuration
    config = MAPFConfig()
    config.parallelizePaths2 = True
    
    # Create solver
    solver = PushAndRotate()
    
    print(f"Initial positions:")
    for actor in actor_set:
        print(f"  Actor {actor.id}: ({actor.current.x}, {actor.current.y}) -> ({actor.goal.x}, {actor.goal.y})")
    
    # Solve
    print("\nSolving...")
    result = solver.start_search(sub_map, config, actor_set)
    
    if result:
        print("✓ Solution found!")
        print(f"Total moves: {len(solver.agents_moves)}")
        
        # Check for collisions
        print("\nChecking for collisions...")
        collision_found = False
        
        # Check position collisions
        for i, move in enumerate(solver.agents_moves):
            agent_id, node = move
            # Check if any other agent is at the same position at the same time
            for j, other_move in enumerate(solver.agents_moves):
                if i != j:
                    other_agent_id, other_node = other_move
                    if node.i == other_node.i and node.j == other_node.j:
                        print(f"✗ Collision detected: Agent {agent_id} and {other_agent_id} at ({node.i}, {node.j})")
                        collision_found = True
        
        # Check edge collisions (agents swapping positions)
        for i in range(len(solver.agents_moves) - 1):
            move1 = solver.agents_moves[i]
            move2 = solver.agents_moves[i + 1]
            agent1_id, node1 = move1
            agent2_id, node2 = move2
            
            # Check if agents are swapping positions
            for j in range(i + 2, len(solver.agents_moves)):
                move3 = solver.agents_moves[j]
                move4 = solver.agents_moves[j + 1] if j + 1 < len(solver.agents_moves) else None
                
                if move4:
                    agent3_id, node3 = move3
                    agent4_id, node4 = move4
                    
                    if (node1.i == node4.i and node1.j == node4.j and 
                        node2.i == node3.i and node2.j == node3.j):
                        print(f"✗ Edge collision detected: Agents {agent1_id} and {agent3_id} swapping positions")
                        collision_found = True
        
        if not collision_found:
            print("✓ No collisions detected!")
        
        # Show final paths
        print("\nFinal paths:")
        for i, path in enumerate(solver.agents_paths):
            if path:
                try:
                    print(f"  Actor {i}: {[(node.i, node.j) for node in path]}")
                except AttributeError:
                    # Handle Point objects
                    print(f"  Actor {i}: {[(node.x, node.y) for node in path]}")
            else:
                print(f"  Actor {i}: No path")
            
    else:
        print("✗ No solution found")

if __name__ == "__main__":
    test_improved_par() 