from simulation import simulate
import argparse

def main(num_sims=10, with_gui=False):
    simulate(num_sims, with_gui)
    

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Dataset Generation")
    parser.add_argument("--num_sims", type=int, default=10, help="Number of simulations")
    parser.add_argument("--with_gui", action="store_true", help="Enable GUI")
    args = parser.parse_args()
    main(args.num_sims, args.with_gui)