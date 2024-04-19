from SymbolicPlanner import SymbolicPlanner
from Predicate import Predicate
import argparse


def generate_predicates(num_blocks):
    """
    Create a list of predicates given a list of blocks
    """
    colors = ["red", "blue", "green", "yellow", "purple", "orange"]
    predicates = []
    
    # For each block determine predicates
    for i in range(num_blocks):
        on_table_predicate = Predicate(f"on-table", [colors[i]], False)
        in_hand_predicate = Predicate(f"in-hand", [colors[i]], False)
        predicates.append(on_table_predicate)
        predicates.append(in_hand_predicate)
        for j in range(num_blocks):
            if i != j:
                on_top_predicate = Predicate(f"on-top", [colors[i], colors[j]], False)
                predicates.append(on_top_predicate)
    
    return predicates


def plan(o, lg, symbolic_planner):
    """
    Get sequence of skills to achieve goal conditions (lg) given
    and observation
    """ 
    initial_conditions = symbolic_planner.get_scene_predicates(o, NUM_BLOCKS)
    return []

def observe():
    """
    Get current point cloud observation
    """
    return None

def execute_plan(o, lg, p, symbolic_planner, max_retrials):
    """
    Execute sequence of skills given by plan (p) to achieve goal conditions (lg) given
    an observation (o)
    """
    retrial_counter = {s: 0 for s in p}
    i = 0
    while i < len(p):
        s = p[i]
        while not s.check_preconditions(o, symbolic_planner, NUM_BLOCKS): # Procondition check
            if i == 0:
                return False
            s = p[i - 1]
        retrial_counter[s] += 1 
        if retrial_counter[s] > max_retrials:
            return False
        
        # Execute skill given obersevation
        s.execute(o)
        new_o = observe()
        if s.check_termination_policy(new_o):
            return True 
        i += 1
    return False

def execute(lg, symbolic_planner, num_blocks, max_replans, max_retrials)->int:
    """
    Execution algorithm
    - lg: Set of goal conditions (predicates)
    """
    predicates = generate_predicates(num_blocks)
    replan_counter = 0
    while replan_counter < max_replans:
        # TODO: Obtain point cloud
        o = observe()
        P = plan(o, lg, symbolic_planner)
        replan_counter += 1
        if execute_plan(o, lg, P, symbolic_planner, max_retrials):
            print("Goal conditions achieved")
            exit(0)
    print("Max replans reached, goal conditions not achieved")
    return exit(1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generizable task planning")
    parser.add_argument("--num_blocks", type=int, default=3, help="Number of blocks")
    parser.add_argument("--max_replans", type=int, default=10, help="Max replans")
    parser.add_argument("--max_retrials", type=int, default=5, help="Max retrials")
    args = parser.parse_args()
    execute([], SymbolicPlanner(), args.num_blocks, args.max_replans, args.max_retrials)
