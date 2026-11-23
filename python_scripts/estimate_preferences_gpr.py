"""Estimate stiffness and neutral angle with scikit-learn Gaussian process regression.

Install in your Python environment: python -m pip install 'scikit-learn>=1.2'
Keep this script beside preference_model_utils.py.

Examples:
    python estimate_preferences_gpr.py  # Train using DATA_FILE below
    python estimate_preferences_gpr.py train other_dataset.mat
    python estimate_preferences_gpr.py predict new_strides.mat --output gpr_estimates.csv
"""

from functools import partial
import math

from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, WhiteKernel
from sklearn.pipeline import make_pipeline

from preference_model_utils import (
    estimator_cli,
    fit_models,
    positive_float,
    predict_joblib_run as predict,
    prepare_training,
    preprocessing_steps,
    run_cli,
    save_joblib_run,
)


# Edit this path to choose the MAT-file to use when no path is passed.
DATA_FILE = (
    r"C:\Users\nazon\OneDrive\Documents\Walter Reed\Preference Project"
    r"\Exploratory Aim Data\Exploratory Aim\datasets\k_norm_by_mass\percent\percent_footshank_dataset.mat"  
)
# Choose "leave_one_out" (by subject) or "data_split" (by stride).
VALIDATION_TYPE = "data_split"
DATA_SPLIT = (80, 10, 10)  # Train, validation, test percentages.


def new_gpr(args, feature_count):
    # Distances between standardized rows typically grow with sqrt(feature_count).
    kernel = RBF(length_scale=args.length_scale * math.sqrt(feature_count))
    kernel += WhiteKernel(noise_level=args.noise)
    return make_pipeline(
        *preprocessing_steps(),
        GaussianProcessRegressor(kernel=kernel, optimizer=None, normalize_y=True),
    )


def train(args):
    features, layout, labels, groups = prepare_training(args)
    model_factory = partial(new_gpr, feature_count=features.shape[1])
    models, report = fit_models(features, layout, labels, groups, args, model_factory)
    save_joblib_run(args.model_dir, models, layout, report, features)


def main():
    parser, fit, _ = estimator_cli(
        __doc__.splitlines()[0], DATA_FILE, VALIDATION_TYPE, DATA_SPLIT, "gpr_results", "gpr_estimates.csv",
    )
    fit.add_argument("--length-scale", type=positive_float, default=1.0, help="RBF length-scale multiplier")
    fit.add_argument("--noise", type=positive_float, default=0.05, help="noise variance in standardized target units")
    run_cli(parser, train, predict)


if __name__ == "__main__":
    main()
