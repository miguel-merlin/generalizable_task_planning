class Skill():
    def __init__(self,
                name,
                logical_preconditions,
                expected_logical_effects,
                visuomotor_policy,
                termination_policy
                ):
        self.name = name
        self.logical_preconditions = logical_preconditions
        self.expected_logical_effects = expected_logical_effects
        self.visuomotor_policy = visuomotor_policy
        self.termination_policy = termination_policy

    def __str__(self):
        return self.name

    def execute(self, o):
        return self.visuomotor_policy(o)