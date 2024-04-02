MAX_REPLANS = 10
MAX_RETRIALS = 10

def plan(o, lg):
    """
    Get sequence of skills to achieve goal conditions (lg) given
    and observation (0)
    """
    return []

def observe():
    """
    Get current point cloud observation
    """
    return None

def execute_plan(o, lg, p):
    """
    Execute sequence of skills given by plan (p) to achieve goal conditions (lg) given
    an observation (o)
    """
    retrial_counter = {s: 0 for s in p}
    i = 0
    while i < len(p):
        s = p[i]
        while not s.check_preconditions(0): # Procondition check
            if i == 0:
                return False
            s = p[i - 1]
        retrial_counter[s] += 1 
        if retrial_counter[s] > MAX_RETRIALS:
            return False
        
        # Execute skill given obersevation
        s.execute(o)
        new_o = observe()
        if s.check_termination_policy(new_o):
            return True
        i += 1
    return False

def execute(lg):
    """
    Execution algorithm
    - lg: Set of goal conditions (predicates)
    """
    replan_counter = 0
    while replan_counter < MAX_REPLANS:
        # TODO: Obtain point cloud
        o = None
        P = plan(o, lg)
        replan_counter += 1
        if execute_plan(o, lg, P):
            return True
    return False
        
        
    