"""Fit boosted-tree regressors to a MAT-file, or predict with saved models.

Install in your Python environment: python -m pip install xgboost scikit-learn
Keep this script beside preference_model_utils.py.

Examples:
    python estimate_preferences_xgboost.py  # Train using DATA_FILE below
    python estimate_preferences_xgboost.py train other_dataset.mat
    python estimate_preferences_xgboost.py predict new_strides.mat --model-dir xgb_results
"""

from preference_model_utils import (
    SEED,
    TARGETS,
    estimator_cli,
    fit_models,
    positive_int,
    prepare_prediction,
    prepare_training,
    run_cli,
    save_run_metadata,
    write_estimates,
)


# Edit this path to choose the MAT-file to use when no path is passed.
DATA_FILE = (
    r"C:\Users\nazon\OneDrive\Documents\Walter Reed\Preference Project"
    r"\Exploratory Aim Data\Exploratory Aim\datasets\delta\delta_footshank_dataset.mat"
)
# Choose "leave_one_out" (by subject) or "data_split" (by stride).
VALIDATION_TYPE = "data_split"
DATA_SPLIT = (80, 10, 10)  # Train, validation, test percentages if using data_split option


def new_model(args):
    from xgboost import XGBRegressor

    return XGBRegressor(
        objective="reg:squarederror",
        booster="gbtree",
        tree_method="hist",
        n_estimators=args.trees,
        learning_rate=0.05,
        max_depth=4,
        max_bin=64,
        subsample=0.85,
        colsample_bytree=0.65,
        random_state=SEED,
        n_jobs=args.jobs,
    )


def train(args):
    features, layout, labels, groups = prepare_training(args)
    models, report = fit_models(features, layout, labels, groups, args, new_model)

    args.model_dir.mkdir(parents=True, exist_ok=True)
    for target, model in models.items():
        model.save_model(args.model_dir / f"{target}.json")
    save_run_metadata(args.model_dir, layout, report)
    write_estimates(args.model_dir / "estimates.csv", models, features)


def predict(args):
    from xgboost import XGBRegressor

    features = prepare_prediction(args)
    models = {}
    for target in TARGETS:
        model = XGBRegressor(n_jobs=args.jobs)
        model.load_model(args.model_dir / f"{target}.json")
        models[target] = model
    write_estimates(args.output, models, features)


def main():
    parser, fit, infer = estimator_cli(
        __doc__.splitlines()[0], DATA_FILE, VALIDATION_TYPE, DATA_SPLIT, "xgb_results", "estimates.csv",
    )
    for command in (fit, infer):
        command.add_argument("--jobs", type=positive_int, default=4)
    fit.add_argument("--trees", type=positive_int, default=200)
    run_cli(parser, train, predict)


if __name__ == "__main__":
    main()
