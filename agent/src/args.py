from argparse import ArgumentParser


def parse_args():
    parser = ArgumentParser("Trains an RL algorithm for autonomous grasping.")

    # logistics
    parser.add_argument("--environment", type=str, default="refinement", help="Environment to load.")
    parser.add_argument("--train", type=int, default=1, help="Whether to train or evaluate the model.")
    parser.add_argument("--seed", type=int, default=0, help="Seed for random number generators.")
    parser.add_argument("--log_name", type=str, default="test", help="Name for log.")
    parser.add_argument("--output_dir", type=str, default="./", help="Path of output directory.")
    parser.add_argument("--chkpt_freq", type=int, default=300, help="Save model every n training steps.")
    parser.add_argument("--check_env", type=int, default=0, help="Whether to check environment using stable baselines checker.")
    parser.add_argument("--algorithm", type=str, default="td3", help="Which algorithm to train with.")
    parser.add_argument("--contribution", type=int, default=1, help="Which contribution of our paper to train/test.")
    parser.add_argument("--log_interval", type=int, default=1, help="After how many episodes to log.")

    # reward framework and reward weights
    parser.add_argument("--framework", type=int, default=1, help="Which reward framework to train with (1 or 2).")
    parser.add_argument("--w_binary_rew", type=float, default=5, help="Weight for binary reward in framework 3")
    parser.add_argument("--w_eps_torque", type=float, default=10, help="Weight for epsilon torque in framework 1 and 3")
    parser.add_argument("--w_delta", type=float, default=0.05, help="Weight for delta in framework 1 and 3")

    # length and update rates of stages
    parser.add_argument("--refine_steps", type=float, default=15, help="Time steps to refine grasp.")
    parser.add_argument("--max_refine_rate", type=float, default=3, help="Max rate of refinment control.")
    parser.add_argument("--lift_steps", type=float, default=6, help="Time steps to lift object.")
    parser.add_argument("--hold_steps", type=float, default=6, help="Time steps to hold object.")
    parser.add_argument("--secs_to_lift", type=float, default=2, help="For how many seconds to lift object.")
    parser.add_argument("--secs_to_hold", type=float, default=2, help="For how many seconds to hold object.")
    parser.add_argument("--lift_dist", type=float, default=0.15, help="How far to lift object.")

    # episode / training end critera
    parser.add_argument("--joint_lim", type=float, default=3, help="End episode if joint limit reached.")
    parser.add_argument("--obj_shift_tol", type=float, default=0.03, help="How far object is allowed to shift.")
    parser.add_argument("--time_steps", type=float, default=2000, help="How many time steps to train.")

    # error added to ground truth wrist pose
    parser.add_argument("--x_error_min", type=float, default=0, help="Positional error along x direction [m]")
    parser.add_argument("--y_error_min", type=float, default=0, help="Positional error along y direction [m]")
    parser.add_argument("--z_error_min", type=float, default=0, help="Positional error along z direction [m]")
    parser.add_argument("--x_error_max", type=float, default=0, help="Positional error along x direction [m]")
    parser.add_argument("--y_error_max", type=float, default=0, help="Positional error along y direction [m]")
    parser.add_argument("--z_error_max", type=float, default=0, help="Positional error along z direction [m]")
    parser.add_argument("--roll_error_min", type=float, default=0, help="Orientational error along x direction [deg]")
    parser.add_argument("--pitch_error_min", type=float, default=0, help="Orientational error along y direction [deg]")
    parser.add_argument("--yaw_error_min", type=float, default=0, help="Orientational error along z direction [deg]")
    parser.add_argument("--roll_error_max", type=float, default=0, help="Orientational error along x direction [deg]")
    parser.add_argument("--pitch_error_max", type=float, default=0, help="Orientational error along y direction [deg]")
    parser.add_argument("--yaw_error_max", type=float, default=0, help="Orientational error along z direction [deg]")

    # evaluation
    parser.add_argument("--eval_freq", type=int, default=10, help="After how many time steps / or episodes to evaluate.")
    parser.add_argument("--eval_after_episode", type=int, default=1, help="Whether to evaluate after time steps or after episodes.")
    parser.add_argument("--n_eval_episodes", type=int, default=10, help="How many episodes to run when evaluating.")
    parser.add_argument("--eval_at_init", type=int, default=1, help="Whether to evaluate the random policy before training.")
    parser.add_argument("--eval_model_path", type=str, help="The path to the model you would like to evaluate.")

    args, unknown = parser.parse_known_args()
    return args
