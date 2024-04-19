from SymbolicPlanner import SymbolicPlanner

class Skill():
    def __init__(self, name, logical_preconditions, logical_effects, visuomotor_policy, termination_condition, num_args) -> None:
        self.name = name
        self.logical_preconditions = logical_preconditions
        self.logical_effects = logical_effects
        self.visuomotor_policy = visuomotor_policy
        self.termination_condition = termination_condition
        self.num_args = num_args
        
    def __str__(self) -> str:
        return f"Skill: {self.name}"

    def __repr__(self) -> str:
        return f"Skill: {self.name}"
    
    def get_num_args(self):
        return self.num_args
    
    def check_preconditions(self, object1, object2=None)-> bool:
        """
        Given list of current enviroment predicates, check if preconditions are satisfied
        """
        predicate_input = ""
        if (object2 is not None):
            predicate_input = f"({object1} {object2})"
        else:
            predicate_input = f"({object1})"
        
        predicate_preconditions = [f"{p}{predicate_input}" for p in self.logical_preconditions]
        print(predicate_preconditions)
        
        return False
    
    def _apply_logical_effects(self):
        """
        Apply logical effects to current state
        """
        return self.logical_effects
    
    def execute(self, o, object_1, object_2):
        """
        Execute skill given observation
        """
        if (object_1 is not None) and (object_2 is not None):
            self.visuomotor_policy.execute(o, object_1, object_2)
            return
        
        if (object_1 is not None) and (object_2 is None):
           self.visuomotor_policy.execute(o, object_1)
           return
    
        raise ValueError("Invalid object arguments for skill execution")
        
        