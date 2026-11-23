"""Estimate stiffness and neutral angle with a small PyTorch neural network.

Install scikit-learn >= 1.2 and PyTorch in your Python environment.
Use the ROCm PyTorch build that matches your runtime for AMD GPU acceleration.
For a CPU-only PyTorch install: pip install torch --index-url https://download.pytorch.org/whl/cpu
Keep this script beside preference_model_utils.py.
Each run reads one MAT-file.

Examples:
    python estimate_preferences_pytorch.py  # Train using DATA_FILE below
    python estimate_preferences_pytorch.py train other_dataset.mat
    python estimate_preferences_pytorch.py train --device rocm
    python estimate_preferences_pytorch.py predict new_strides.mat
"""

from dataclasses import dataclass

import joblib
import numpy as np
from sklearn.pipeline import Pipeline, make_pipeline
import torch
from torch import nn

from preference_model_utils import (
    SEED,
    TARGETS,
    estimator_cli,
    make_data_split,
    positive_int,
    prepare_prediction,
    prepare_training,
    preprocessing_steps,
    run_cli,
    save_run_metadata,
    split_report,
    subject_folds,
    write_estimate_rows,
)


# Edit this path to choose the MAT-file to use when no path is passed.
DATA_FILE = (
    r"C:\Users\nazon\OneDrive\Documents\Walter Reed\Preference Project"
    r"\Exploratory Aim Data\Exploratory Aim\datasets\k_norm_by_mass\percent\percent_footshank_dataset.mat"  
)
# Choose "leave_one_out" (by subject) or "data_split" (by stride).
VALIDATION_TYPE = "data_split"
DATA_SPLIT = (80, 10, 10)  # Train, validation, test percentages.


def build_network(feature_count, hidden_width):
    return nn.Sequential(
        nn.Linear(feature_count, hidden_width),
        nn.ReLU(),
        nn.Linear(hidden_width, len(TARGETS)),
    )


def select_device(requested):
    """Use the PyTorch HIP backend when it can run on the visible GPU."""
    cpu = torch.device("cpu")
    if requested == "cpu":
        print("Using CPU.", flush=True)
        return cpu

    hip_version = torch.version.hip
    reason = "The installed PyTorch build does not include ROCm."
    if hip_version:
        reason = "PyTorch cannot see a ROCm GPU."
        try:
            if torch.cuda.is_available():
                # PyTorch exposes HIP devices through its cuda namespace.
                device = torch.device("cuda", torch.cuda.current_device())
                probe = torch.ones((2, 2), device=device)
                (probe @ probe).cpu()
                print(f"Using ROCm {hip_version} on {torch.cuda.get_device_name(device)}.", flush=True)
                return device
        except RuntimeError as error:
            reason = f"ROCm GPU initialization failed. {error}"
    if requested == "rocm":
        raise RuntimeError(reason)
    print(f"{reason} Using CPU.", flush=True)
    return cpu


@dataclass
class FittedNetwork:
    network: nn.Module
    preprocessing: Pipeline
    target_mean: np.ndarray
    target_scale: np.ndarray

    @torch.inference_mode()
    def predict(self, features):
        self.network.eval()
        inputs = self.preprocessing.transform(features).astype(np.float32, copy=False)
        device = next(self.network.parameters()).device
        normalized = self.network(torch.from_numpy(inputs).to(device)).cpu().numpy()
        estimates = normalized * self.target_scale + self.target_mean
        if not np.isfinite(estimates).all():
            raise ValueError("The neural network produced nonfinite estimates.")
        return estimates


def fit_network(features, targets, args, device):
    observed = np.isfinite(targets)
    labeled = observed.any(axis=1)
    targets, observed = targets[labeled], observed[labeled]

    preprocessing = make_pipeline(*preprocessing_steps())
    inputs = preprocessing.fit_transform(features[labeled]).astype(np.float32, copy=False)

    # Give each target comparable weight and mask missing labels out of the loss.
    counts = np.maximum(observed.sum(axis=0), 1)
    target_mean = np.where(observed, targets, 0).sum(axis=0) / counts
    centered = np.where(observed, targets - target_mean, 0)
    target_scale = np.sqrt((centered**2).sum(axis=0) / counts)
    target_scale[target_scale == 0] = 1

    inputs = torch.from_numpy(inputs).to(device)
    normalized = torch.from_numpy((centered / target_scale).astype(np.float32)).to(device)
    observed = torch.from_numpy(observed).to(device)
    torch.manual_seed(SEED)
    generator = torch.Generator(device=device).manual_seed(SEED)
    network = build_network(features.shape[1], args.hidden_width).to(device)
    optimizer = torch.optim.AdamW(network.parameters(), lr=0.001)
    for _ in range(args.epochs):
        batches = torch.randperm(len(inputs), device=device, generator=generator).split(args.batch_size)
        for batch in batches:
            optimizer.zero_grad(set_to_none=True)
            residual = network(inputs[batch]) - normalized[batch]
            mask = observed[batch]
            loss = residual.masked_fill(~mask, 0).square().sum() / mask.sum()
            loss.backward()
            optimizer.step()
    return FittedNetwork(network, preprocessing, target_mean, target_scale)


def grouped_validation(features, targets, groups, args, device):
    validation = dict.fromkeys(TARGETS)
    if groups is None or args.skip_validation:
        return validation

    observed = np.isfinite(targets)
    eligible = np.flatnonzero(observed.any(axis=1) & np.isfinite(groups))
    subject_count = len(np.unique(groups[eligible]))
    if subject_count < 2:
        return validation

    errors = np.full(targets.shape, np.nan)
    fold_counts = np.zeros(len(TARGETS), dtype=int)
    for train_rows, test_rows in subject_folds(eligible, groups):
        can_score = observed[train_rows].any(axis=0) & observed[test_rows].any(axis=0)
        if can_score.any():
            model = fit_network(features[train_rows], targets[train_rows], args, device)
            residual = model.predict(features[test_rows]) - targets[test_rows]
            errors[test_rows] = np.where(can_score, residual, np.nan)
            fold_counts += can_score

    for index, target in enumerate(TARGETS):
        evaluated = np.isfinite(errors[:, index])
        if evaluated.any():
            residual = errors[evaluated, index]
            validation[target] = {
                "mae": float(np.abs(residual).mean()),
                "rmse": float(np.sqrt((residual**2).mean())),
                "rows": int(evaluated.sum()),
                "subjects": len(np.unique(groups[evaluated])),
                "folds": int(fold_counts[index]),
            }
    return validation


def configure_device(args):
    torch.set_num_threads(args.jobs)
    return select_device(args.device)


def train(args, device=None):
    if device is None:
        device = configure_device(args)
    features, layout, labels, groups = prepare_training(args)
    targets = np.column_stack([labels[key] for key in TARGETS]).astype(np.float64)
    print(f"Training with {len(features):,} rows and {features.shape[1]:,} feature values per row.", flush=True)

    split = make_data_split(labels, args)
    report = {}
    if split is None:
        scores = grouped_validation(features, targets, groups, args, device)
        for target, score in scores.items():
            report[target] = {"labeled_rows": int(np.isfinite(labels[target]).sum()), "validation": score}
            if score is not None:
                print(f"{target} held-out subject MAE = {score['mae']:.4g}, RMSE = {score['rmse']:.4g}", flush=True)
        train_rows = slice(None)
    else:
        train_rows = split["train"]
    model = fit_network(features[train_rows], targets[train_rows], args, device)
    estimates = model.predict(features)
    if split is not None:
        for index, target in enumerate(TARGETS):
            report[target] = split_report(target, labels[target], split, lambda rows: estimates[rows, index])
    args.model_dir.mkdir(parents=True, exist_ok=True)
    torch.save({
        "state_dict": {name: value.cpu() for name, value in model.network.state_dict().items()},
        "hidden_width": args.hidden_width,
        "target_mean": torch.from_numpy(model.target_mean),
        "target_scale": torch.from_numpy(model.target_scale),
    }, args.model_dir / "network.pt")
    joblib.dump(model.preprocessing, args.model_dir / "preprocessing.joblib")
    save_run_metadata(args.model_dir, layout, report)
    write_estimate_rows(args.model_dir / "estimates.csv", estimates, len(features))


def predict(args, device=None):
    if device is None:
        device = configure_device(args)
    features = prepare_prediction(args)
    checkpoint = torch.load(args.model_dir / "network.pt", weights_only=True, map_location="cpu")
    network = build_network(features.shape[1], checkpoint["hidden_width"])
    network.load_state_dict(checkpoint["state_dict"])
    model = FittedNetwork(
        network.to(device),
        joblib.load(args.model_dir / "preprocessing.joblib"),
        checkpoint["target_mean"].numpy(),
        checkpoint["target_scale"].numpy(),
    )
    write_estimate_rows(args.output, model.predict(features), len(features))


def main():
    parser, fit, infer = estimator_cli(
        __doc__.splitlines()[0], DATA_FILE, VALIDATION_TYPE, DATA_SPLIT, "pytorch_results", "pytorch_estimates.csv",
    )
    for command in (fit, infer):
        command.add_argument("--jobs", type=positive_int, default=4, help="PyTorch CPU threads")
        command.add_argument(
            "--device", choices=("auto", "cpu", "rocm"), default="auto",
            help="auto uses a working ROCm GPU when available and otherwise uses CPU",
        )
    fit.add_argument("--epochs", type=positive_int, default=40)
    fit.add_argument("--batch-size", type=positive_int, default=128)
    fit.add_argument("--hidden-width", type=positive_int, default=64)
    run_cli(parser, train, predict)


if __name__ == "__main__":
    main()
