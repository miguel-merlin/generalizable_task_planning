from simulation.simulation import simulate
import argparse

def main(num_sims=10):
    simulate(num_sims)
    

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Dataset Generation")
    parser.add_argument("--num_sims", type=int, default=10, help="Number of simulations")
    args = parser.parse_args()
    main(args.num_sims)