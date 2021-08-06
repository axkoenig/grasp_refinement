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
    parser.add_argument("--algorithm", type=str, default="sac", help="Which algorithm to train with.")
    parser.add_argument("--log_interval", type=int, default=1, help="After how many episodes to log.")
    parser.add_argument("--all_test_cases", type=int, default=1, help="Whether or not to run on subset of test cases with larger wrist errors.")
    parser.add_argument("--simplify_collisions", type=int, default=1, help="Whether or not to use simplified collision meshes of robotic hand.")

    # frameworks and weights
    parser.add_argument("--reward_framework", type=int, default=1, help="(1=epsilon+delta), (2=delta), (3=epsilon), (4=binary).")
    parser.add_argument("--force_framework", type=int, default=1, help="(1=full), (2=normal), (3=binary), (4=none).")
    parser.add_argument("--torque_framework", type=int, default=3, help="(1=perfect), (2=noisy), (3=none).")
    parser.add_argument("--w_eps_torque", type=float, default=5, help="Weight for epsilon torque")
    parser.add_argument("--w_delta", type=float, default=0.5, help="Weight for delta")
    parser.add_argument("--torque_noise", type=float, default=0.5, help="Noise added to torque measurement as a percentage of max torque.")

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
    parser.add_argument("--obj_shift_tol", type=float, default=0.1, help="How far object is allowed to shift.")
    parser.add_argument("--time_steps", type=float, default=2000, help="How many time steps to train.")

    # error added to ground truth wrist pose
    parser.add_argument("--x_error_min", type=float, default=-0.05, help="Positional error along x direction [m]")
    parser.add_argument("--y_error_min", type=float, default=-0.05, help="Positional error along y direction [m]")
    parser.add_argument("--z_error_min", type=float, default=-0.05, help="Positional error along z direction [m]")
    parser.add_argument("--x_error_max", type=float, default=0.05, help="Positional error along x direction [m]")
    parser.add_argument("--y_error_max", type=float, default=0.05, help="Positional error along y direction [m]")
    parser.add_argument("--z_error_max", type=float, default=0.05, help="Positional error along z direction [m]")
    parser.add_argument("--roll_error_min", type=float, default=-10, help="Orientational error along x direction [deg]")
    parser.add_argument("--pitch_error_min", type=float, default=-10, help="Orientational error along y direction [deg]")
    parser.add_argument("--yaw_error_min", type=float, default=-10, help="Orientational error along z direction [deg]")
    parser.add_argument("--roll_error_max", type=float, default=10, help="Orientational error along x direction [deg]")
    parser.add_argument("--pitch_error_max", type=float, default=10, help="Orientational error along y direction [deg]")
    parser.add_argument("--yaw_error_max", type=float, default=10, help="Orientational error along z direction [deg]")

    # evaluation
    parser.add_argument("--gen_new_test_cases", type=int, default=0, help="Whether to generate new test cases for evaluation.")
    parser.add_argument("--eval_during_training", type=int, default=0, help="Whether to evaluate using callback during training.")
    parser.add_argument("--eval_after_training", type=int, default=1, help="Whether to evaluate test cases after training.")
    parser.add_argument("--eval_freq", type=int, default=10, help="After how many time steps / or episodes to evaluate.")
    parser.add_argument("--eval_after_episode", type=int, default=1, help="Whether to evaluate after eval_freq time steps or after episodes.")
    parser.add_argument("--n_eval_episodes", type=int, default=10, help="How many episodes to run when evaluating.")
    parser.add_argument("--eval_at_init", type=int, default=1, help="Whether to evaluate the random policy before training.")
    parser.add_argument("--test_model_path", type=str, help="The path to the model you would like to test.")
    parser.add_argument("--deterministic_eval", type=int, default=1, help="Whether model evaluation should be deterministic.")

    # td3 hparam
    parser.add_argument("--policy_delay", type=int, default=10, help="Q values will be updated policy_delay more often than policy for TD3.")

    # sac hparam
    parser.add_argument("--ent_coef", type=float, default=0.01, help="Fixed entropy regularization coefficient for SAC.")

    # td3 and sac hparams
    parser.add_argument("--learning_starts", type=int, default=100, help="How many data points to collect before starting to train.")
    parser.add_argument("--batch_size", type=int, default=64, help="Minibatch size for each gradient update.")
    parser.add_argument("--gradient_steps", type=int, default=32, help="How many gradient steps to do after each rollout.")
    parser.add_argument("--train_freq", type=int, default=32, help="Update the model every train_freq time steps.")
    parser.add_argument("--learning_rate", type=float, default=0.0001, help="Learning rate for ADAM optimizer.")
    parser.add_argument("--use_lr_schedule", type=int, default=0, help="Whether to use a learning rate scheduler or not.")
    parser.add_argument("--tau", type=float, default=0.001, help="Soft update coefficient (between 0 and 1).")

    args, unknown = parser.parse_known_args()
    return args
