from simulation import simulate
import argparse

def main(num_sims=10, num_objects=1, sim_type = 0, with_gui=False):
    simulate(num_sims, num_objects, sim_type, with_gui)
    

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Dataset Generation")
    parser.add_argument("--num_sims", type=int, default=10, help="Number of simulations")
    parser.add_argument("--with_gui", action="store_true", help="Enable GUI")
    #TO DO: add num objects arg and simtype arg

    args = parser.parse_args()
    main(num_sims=args.num_sims, with_gui=args.with_gui) 