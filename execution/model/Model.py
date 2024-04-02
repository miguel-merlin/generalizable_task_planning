class Model:
    def __init__(self, name, symbolic_planner, predicates):
        self.name = name
        self.skills = symbolic_planner
        self.predicates = predicates
    
    def get_scene_predicates(o, objects):
        """
        Given an observation return vector of predicates
        """
        return []