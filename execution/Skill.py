from SymbolicPlanner import SymbolicPlanner

class Skill():
    def __init__(self, name, logical_preconditions, logical_effects, visuomotor_policy, termination_condition) -> None:
        self.name = name
        self.logical_preconditions = logical_preconditions
        self.logical_effects = logical_effects
        self.visuomotor_policy = visuomotor_policy
        self.termination_condition = termination_condition
    
    def check_preconditions(self, o, synbolic_planner: SymbolicPlanner, num_blocks):
        """
        Given point cloud observation, check if preconditions are met
        - o: Point cloud observation
        - symbolic_planner: Symbolic planner for scene predicates
        """
        scene_predicates = synbolic_planner.get_scene_predicates(o, num_blocks)
        
        # Convert list of binary predicates to binary number
        scene_predicates = [int(p) for p in scene_predicates]
        scene_predicates = int(''.join(map(str, scene_predicates)), 2)
        
        # Convert logical preconditions to binary number
        logical_preconditions = [int(p) for p in self.logical_preconditions]
        logical_preconditions = int(''.join(map(str, logical_preconditions)), 2)
        
        # AND logical operation between scene predicates and logical preconditions
        return scene_predicates & logical_preconditions == logical_preconditions
        