from tensorboard import program
import argparse


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Launch TensorBoard with a specified log directory."
    )
    parser.add_argument(
        "--logdir",
        type=str,
        required=True,
        help="Path to the TensorBoard log directory.",
    )
    args = parser.parse_args()
    logdir = args.logdir

    tb = program.TensorBoard()
    tb.configure(argv=[None, "--logdir", logdir])
    url = tb.main()
    print(f"Tensorflow listening on {url}")
