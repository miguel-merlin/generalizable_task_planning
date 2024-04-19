class Predicate:
    def __init__(self, predicate_string, arguments, condition) -> None:
        self.predicate_string = predicate_string
        self.arguments = arguments
        self.condition = condition
    
    def __bool__(self) -> bool:
        return self.conditions
    
    def __str__(self) -> str:
        return f"{self.predicate_string}({', '.join(self.arguments)}): {self.condition}"