from Model import Model

class SymbolicPlanner(Model):
    def __init__(self, model) -> None:
        super().__init__(model)
    
    def get_scene_predicates(o, num_blocks):
        """
        Get scene predicates
        - o: Point cloud observation
        - num_blocks: Number of blocks
        """
        return []