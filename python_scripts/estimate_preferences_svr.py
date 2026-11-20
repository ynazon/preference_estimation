"""Estimate stiffness and neutral angle with scikit-learn support vector regression.

Install in your Python environment: python -m pip install 'scikit-learn>=1.2'
Keep this script beside preference_model_utils.py.

Examples:
    python estimate_preferences_svr.py  # Train using DATA_FILE below
    python estimate_preferences_svr.py train other_dataset.mat
    python estimate_preferences_svr.py predict new_strides.mat --output svr_estimates.csv
"""

import argparse
import math

from sklearn.compose import TransformedTargetRegressor
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler
from sklearn.svm import SVR

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
    r"\Exploratory Aim Data\Exploratory Aim\datasets\delta\delta_footshank_dataset.mat"
)
# Choose "leave_one_out" (by subject) or "data_split" (by stride).
VALIDATION_TYPE = "data_split"
DATA_SPLIT = (80, 10, 10)  # Train, validation, test percentages.


def new_svr(args):
    regressor = make_pipeline(
        *preprocessing_steps(),
        SVR(kernel="rbf", C=args.c, epsilon=args.epsilon, gamma="scale"),
    )
    return TransformedTargetRegressor(regressor=regressor, transformer=StandardScaler())


def nonnegative_float(value):
    number = float(value)
    if not math.isfinite(number) or number < 0:
        raise argparse.ArgumentTypeError("must be finite and nonnegative")
    return number


def train(args):
    features, layout, labels, groups = prepare_training(args)
    models, report = fit_models(features, layout, labels, groups, args, new_svr)
    save_joblib_run(args.model_dir, models, layout, report, features)


def main():
    parser, fit, _ = estimator_cli(
        __doc__.splitlines()[0], DATA_FILE, VALIDATION_TYPE, DATA_SPLIT, "svr_results", "svr_estimates.csv",
    )
    fit.add_argument("--c", type=positive_float, default=10.0, help="SVR regularization parameter")
    fit.add_argument("--epsilon", type=nonnegative_float, default=0.1, help="SVR margin in standardized target units")
    run_cli(parser, train, predict)


if __name__ == "__main__":
    main()
