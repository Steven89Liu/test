from simulation_issue import Simulation

from argparse import ArgumentParser

import pybullet

if not pybullet.isNumpyEnabled():
    raise RuntimeError(
        "PyBullet must have been compiled with NumPy support. Try upgradinng it or install from source."
    )


'''
Drift demonstration script

It creates a simulator object in a thread and runs. During running, the
motor of the gantry is accelerated for n cycles and after reaching the
threshold it is de-accelerated. After stopping, there should be a drift
visible in the GUI. The drift does not stop, despite the fact,
friction is enabled.

'''



def main():
    parser = ArgumentParser()

    parser.add_argument(
        "-g",
        "--gui",
        dest="gui",
        action="store_true",
        default=True,
        help="Show the GUI or run headless.",
    )

    parser.add_argument(
        "-r",
        "--realistic-speed",
        action="store_true",
        default=True,
        help="When given, slow down the crane so it roughly matches the real crane",
    )


    parser.add_argument(
        "-s",
        "--nosway",
        action="store_false",
        default=True,
        help="Disable swayable hoist.",
    )

    args = parser.parse_args()
    sim = Simulation(
        gui=args.gui,
        use_realistic_speed=args.realistic_speed,
        en_sway = args.nosway,
    )
    sim.run()


if __name__ == "__main__":
    main()
