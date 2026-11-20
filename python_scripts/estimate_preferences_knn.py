"""Estimate stiffness and neutral angle with scikit-learn k-nearest neighbors.

Install in your Python environment: python -m pip install 'scikit-learn>=1.2'
Keep this script beside preference_model_utils.py.

Examples:
    python estimate_preferences_knn.py  # Train using DATA_FILE below
    python estimate_preferences_knn.py train other_dataset.mat
    python estimate_preferences_knn.py predict new_strides.mat --output knn_estimates.csv
"""

from sklearn.neighbors import KNeighborsRegressor
from sklearn.pipeline import make_pipeline

from preference_model_utils import (
    estimator_cli,
    fit_models,
    positive_int,
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


def new_knn(args):
    return make_pipeline(
        *preprocessing_steps(),
        KNeighborsRegressor(
            n_neighbors=args.neighbors, weights="distance", metric="euclidean", algorithm="brute",
        ),
    )


def train(args):
    features, layout, labels, groups = prepare_training(args)
    models, report = fit_models(features, layout, labels, groups, args, new_knn)
    save_joblib_run(args.model_dir, models, layout, report, features)


def main():
    parser, fit, _ = estimator_cli(
        __doc__.splitlines()[0], DATA_FILE, VALIDATION_TYPE, DATA_SPLIT, "knn_results", "knn_estimates.csv",
    )
    fit.add_argument("--neighbors", type=positive_int, default=5, help="number of nearest neighbors")
    run_cli(parser, train, predict)


if __name__ == "__main__":
    main()
