"""Shared MAT-file, validation, and output utilities for the preference estimators."""

import argparse
import csv
import json
import os
from pathlib import Path, PureWindowsPath
import shutil
import subprocess
import sys
import tempfile

import joblib
import numpy as np
from scipy.io import loadmat
from sklearn.impute import SimpleImputer
from sklearn.model_selection import LeaveOneGroupOut
from sklearn.preprocessing import StandardScaler


TARGETS = ("HT_stiffness", "HT_neutral_angle")
EXCLUDED = {"Subject", "Exo_Side", "Time", "Trial_Number", *TARGETS}
SEED = 42


# Preserve each table variable as an N-by-width block.
MATLAB_EXPORT = r"""
function preference_xgb_export(source, variable, destination, training)
    loaded = load(source, variable);
    assert(isfield(loaded, variable), 'MAT-file has no variable named "%s".', variable);
    t = loaded.(variable);
    clear loaded
    assert(istable(t) && height(t) > 0, 'Expected a nonempty MATLAB table.');

    targets = {'HT_stiffness', 'HT_neutral_angle'};
    omitted = [{'Subject', 'Exo_Side', 'Time', 'Trial_Number'}, targets];
    names = setdiff(t.Properties.VariableNames, omitted, 'stable');
    assert(~isempty(names), 'No predictor columns are available.');
    if training
        assert(all(ismember(targets, t.Properties.VariableNames)), ...
            'Training requires both target columns.');
        names = [names, targets];
    end

    binary = fopen(fullfile(destination, 'columns.bin'), 'w', 'ieee-le');
    assert(binary >= 0, 'Could not open the temporary feature file.');
    closeFile = onCleanup(@() fclose(binary));
    widths = zeros(1, numel(names));
    for k = 1:numel(names)
        values = t.(names{k});
        assert((isnumeric(values) || islogical(values)) && isreal(values) && ...
            ismatrix(values) && size(values, 2) > 0, ...
            'Column "%s" must contain real numeric or logical values.', names{k});
        widths(k) = size(values, 2);
        written = fwrite(binary, full(single(values)), 'single');
        assert(written == numel(values), 'Could not write all values for "%s".', names{k});
    end
    hasSubject = training && ismember('Subject', t.Properties.VariableNames);
    if hasSubject
        groups = findgroups(string(t.Subject));
        assert(iscolumn(groups), 'Subject must have one value per row.');
        assert(fwrite(binary, single(groups), 'single') == height(t), ...
            'Could not write the subject groups.');
    end
    clear closeFile

    meta = struct('rows', height(t), 'names', {names}, 'widths', widths, ...
        'hasSubject', hasSubject);
    file = fopen(fullfile(destination, 'schema.json'), 'w', 'n', 'UTF-8');
    assert(file >= 0, 'Could not open the temporary schema file.');
    closeSchema = onCleanup(@() fclose(file));
    fprintf(file, '%s', jsonencode(meta));
end
"""


def read_matlab_table(path, variable, matlab, training):
    executable = matlab or shutil.which("matlab") or shutil.which("matlab.exe")
    if executable is None and "WSL_DISTRO_NAME" in os.environ:
        windows_matlab = sorted(Path("/mnt/c/Program Files/MATLAB").glob("R*/bin/matlab.exe"))
        if windows_matlab:
            executable = str(windows_matlab[-1])
    if executable is None:
        raise ValueError("MATLAB is needed to read this table. Pass --matlab /path/to/matlab.")

    windows_matlab = os.name == "nt" or str(executable).lower().endswith(".exe")
    running_in_wsl = os.name != "nt" and windows_matlab

    def matlab_path(local_path):
        value = str(local_path)
        if running_in_wsl:
            result = subprocess.run(
                ["wslpath", "-w", value], check=True, capture_output=True, text=True
            )
            value = result.stdout.strip()
        return value

    def literal(value):
        return "'" + value.replace("'", "''") + "'"

    with tempfile.TemporaryDirectory(prefix="preference_xgb_") as directory:
        scratch = Path(directory).resolve()
        (scratch / "preference_xgb_export.m").write_text(MATLAB_EXPORT, encoding="utf-8")
        statement = (
            f"addpath({literal(matlab_path(scratch))}, '-begin'); "
            f"preference_xgb_export({literal(matlab_path(path))}, {literal(variable)}, "
            f"{literal(matlab_path(scratch))}, {str(training).lower()});"
        )
        log = scratch / "matlab.log"
        command = [executable, "-batch", statement, "-logfile", matlab_path(log)]
        if windows_matlab:
            command.insert(1, "-wait")
        completed = subprocess.run(command, capture_output=True, errors="replace")
        if completed.returncode != 0 or not (scratch / "schema.json").exists():
            detail = log.read_text(encoding="utf-8", errors="replace") if log.exists() else (
                completed.stderr or completed.stdout
            )
            raise ValueError(f"MATLAB could not read the input table.\n{detail[-3000:]}")

        schema = json.loads((scratch / "schema.json").read_text(encoding="utf-8"))
        widths = np.atleast_1d(schema["widths"]).astype(int)
        offsets = np.r_[0, widths.cumsum()]
        rows = schema["rows"]
        blocks = offsets[-1] + int(schema["hasSubject"])
        values = np.fromfile(scratch / "columns.bin", dtype="<f4").reshape(rows, blocks, order="F")
        columns = {
            name: values[:, start:stop]
            for name, start, stop in zip(schema["names"], offsets[:-1], offsets[1:])
        }
        if schema["hasSubject"]:
            columns["Subject"] = values[:, -1:]
        return columns


def read_input(args, training):
    path = args.matfile
    if os.name != "nt" and os.environ.get("WSL_DISTRO_NAME"):
        if PureWindowsPath(str(path)).is_absolute():
            converted = subprocess.run(
                ["wslpath", "-u", str(path)], check=True, capture_output=True, text=True
            )
            path = Path(converted.stdout.strip())
    path = path.expanduser().resolve(strict=True)
    if args.plain_mat:
        contents = loadmat(path, struct_as_record=False, variable_names=[args.variable])
        if args.variable not in contents:
            raise ValueError(f"MAT-file has no variable named {args.variable!r}.")
        structs = contents[args.variable]
        if structs.size != 1 or not hasattr(structs.item(), "_fieldnames"):
            raise ValueError("--plain-mat expects a scalar MATLAB struct of column arrays.")
        struct = structs.item()
        return {name: getattr(struct, name) for name in struct._fieldnames}
    return read_matlab_table(path, args.variable, args.matlab, training)


def feature_matrix(columns, expected=None):
    layout = {}
    for name, values in columns.items():
        if name in EXCLUDED:
            continue
        if values.ndim != 2 or not values.size or values.dtype.kind not in "biuf":
            raise ValueError(f"Predictor {name!r} must be a nonempty real numeric matrix.")
        layout[name] = values.shape[1]
    if not layout:
        raise ValueError("No predictor columns are available.")
    if expected is not None and layout != expected:
        missing = expected.keys() - layout.keys()
        extra = layout.keys() - expected.keys()
        resized = [name for name in expected.keys() & layout.keys() if expected[name] != layout[name]]
        raise ValueError(
            f"Predictor schema differs from training. Missing: {sorted(missing)}. "
            f"Extra: {sorted(extra)}. Different width: {sorted(resized)}."
        )
    order = expected if expected is not None else layout
    try:
        features = np.concatenate([columns[name] for name in order], axis=1, dtype=np.float32)
    except ValueError as error:
        raise ValueError("All predictor variables must contain the same number of rows.") from error
    features[~np.isfinite(features)] = np.nan
    return np.ascontiguousarray(features), layout


def add_validation_options(parser, validation_type, data_split):
    parser.add_argument(
        "--validation-type", "--validation_type", default=validation_type,
        choices=("leave_one_out", "data_split"),
        help="leave one subject out or split strides into train, validation, and test sets",
    )
    parser.add_argument(
        "--split-ratios", dest="data_split", type=float, nargs=3, default=data_split,
        metavar=("TRAIN", "VALIDATION", "TEST"),
        help="percentages for data_split (default: %(default)s)",
    )
    parser.add_argument(
        "--skip-validation", action="store_true", help="fit all labeled rows without validation or testing",
    )


def make_data_split(labels, args):
    """Return one reproducible stride partition shared by both targets."""
    if args.skip_validation or getattr(args, "validation_type", "leave_one_out") != "data_split":
        return None
    ratios = np.asarray(args.data_split, dtype=float)
    if (ratios.shape != (3,) or not np.isfinite(ratios).all()
            or np.any(ratios <= 0) or not np.isclose(ratios.sum(), 100, rtol=0, atol=1e-8)):
        raise ValueError("DATA_SPLIT needs three positive percentages that sum to 100.")

    observed = {target: np.isfinite(labels[target]) for target in TARGETS}
    eligible = np.flatnonzero(np.logical_or.reduce(tuple(observed.values())))
    if len(eligible) < 3:
        raise ValueError("Data split requires at least three labeled strides.")
    expected = len(eligible) * ratios / ratios.sum()
    counts = np.floor(expected).astype(int)
    remainder_order = np.argsort(counts - expected, kind="stable")
    counts[remainder_order[:len(eligible) - counts.sum()]] += 1
    for empty_split in np.flatnonzero(counts == 0):
        counts[np.argmax(counts)] -= 1
        counts[empty_split] = 1

    shuffled = np.random.default_rng(SEED).permutation(eligible)
    train, validation, test = np.split(shuffled, np.cumsum(counts)[:-1])
    for target, available in observed.items():
        if not available[train].any():
            raise ValueError(f"The data split has no training labels for {target}. Try a larger training percentage.")
    print(f"Data split uses {len(train):,} training, {len(validation):,} validation, "
          f"and {len(test):,} test strides.", flush=True)
    return {"train": train, "validation": validation, "test": test}


def split_report(target, labels, split, predict_rows):
    result = {
        "labeled_rows": int(np.isfinite(labels).sum()),
        "training_rows": int(np.isfinite(labels[split["train"]]).sum()),
        "split_rows": {name: len(rows) for name, rows in split.items()},
    }
    for name in ("validation", "test"):
        rows = split[name]
        rows = rows[np.isfinite(labels[rows])]
        result[name] = None
        if len(rows):
            errors = np.asarray(predict_rows(rows), dtype=np.float64) - labels[rows]
            if not np.isfinite(errors).all():
                raise ValueError(f"Nonfinite {name} estimates for {target}.")
            result[name] = {
                "mae": float(np.abs(errors).mean()),
                "rmse": float(np.sqrt((errors**2).mean())),
                "rows": len(rows),
            }
            score = result[name]
            print(f"{target} {name} MAE = {score['mae']:.4g}, RMSE = {score['rmse']:.4g}", flush=True)
    return result


def subject_folds(eligible, groups):
    """Yield original row indices for each leave-one-subject-out fold."""
    for train, held_out in LeaveOneGroupOut().split(eligible, groups=groups[eligible]):
        yield eligible[train], eligible[held_out]


def grouped_validation(features, labels, groups, args, model_factory):
    if groups is None or args.skip_validation:
        return None
    eligible = np.flatnonzero(np.isfinite(labels) & np.isfinite(groups))
    subjects = len(np.unique(groups[eligible]))
    if subjects < 2:
        return None

    errors = np.empty(len(labels), dtype=np.float64)
    for train_rows, test_rows in subject_folds(eligible, groups):
        model = model_factory(args)
        model.fit(features[train_rows], labels[train_rows])
        errors[test_rows] = model.predict(features[test_rows]) - labels[test_rows]
    errors = errors[eligible]
    return {
        "mae": float(np.mean(np.abs(errors))),
        "rmse": float(np.sqrt(np.mean(errors**2))),
        "rows": len(eligible),
        "subjects": subjects,
        "folds": subjects,
    }


def write_estimate_rows(path, estimates, row_count):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(TARGETS)
        writer.writerows(estimates)
    print(f"Wrote {row_count:,} rows to {path}")


def write_estimates(path, models, features):
    predictions = [models[target].predict(features) for target in TARGETS]
    write_estimate_rows(path, zip(*predictions), len(features))


def training_labels(columns, row_count):
    labels = {}
    for target in TARGETS:
        values = columns.get(target)
        if values is None or values.shape != (row_count, 1) or values.dtype.kind not in "biuf":
            raise ValueError(f"Training requires one numeric value per row for {target}.")
        labels[target] = values[:, 0]
        if not np.isfinite(labels[target]).any():
            raise ValueError(f"No finite training labels for {target}.")

    groups = columns.get("Subject")
    if groups is not None:
        if groups.shape != (row_count, 1) or groups.dtype.kind not in "biuf":
            raise ValueError("Subject must have one identifier per row (numeric with --plain-mat).")
        groups = groups[:, 0]

    return labels, groups


def fit_models(features, layout, labels, groups, args, model_factory):
    print(f"Training with {len(features):,} rows, {len(layout)} predictor variables, "
          f"and {features.shape[1]:,} feature values per row.", flush=True)
    split = make_data_split(labels, args)
    models = {}
    report = {}
    for target in TARGETS:
        values = labels[target]
        valid = np.isfinite(values)
        train_rows = np.flatnonzero(valid) if split is None else split["train"][valid[split["train"]]]
        print(f"Fitting {target} on {len(train_rows):,} labeled rows.", flush=True)
        if split is None:
            validation = grouped_validation(features, values, groups, args, model_factory)
            if validation is None:
                print("Subject-held-out validation skipped or fewer than two labeled subjects.")
            else:
                print(f"Subject-held-out MAE = {validation['mae']:.4g}, "
                      f"RMSE = {validation['rmse']:.4g}", flush=True)
            report[target] = {"labeled_rows": int(valid.sum()), "validation": validation}
        model = model_factory(args)
        model.fit(features[train_rows], values[train_rows])
        models[target] = model
        if split is not None:
            report[target] = split_report(target, values, split, lambda rows: model.predict(features[rows]))
    return models, report


def positive_int(value):
    number = int(value)
    if number < 1:
        raise argparse.ArgumentTypeError("must be at least 1")
    return number


def positive_float(value):
    number = float(value)
    if not np.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be finite and positive")
    return number


def preprocessing_steps():
    return SimpleImputer(strategy="median", keep_empty_features=True), StandardScaler()


def prepare_training(args):
    columns = read_input(args, training=True)
    features, layout = feature_matrix(columns)
    labels, groups = training_labels(columns, len(features))
    return features, layout, labels, groups


def prepare_prediction(args):
    layout = json.loads((args.model_dir / "features.json").read_text(encoding="utf-8"))
    columns = read_input(args, training=False)
    features, _ = feature_matrix(columns, expected=layout)
    return features


def save_run_metadata(model_dir, layout, report):
    (model_dir / "features.json").write_text(json.dumps(layout, indent=2), encoding="utf-8")
    (model_dir / "metrics.json").write_text(json.dumps(report, indent=2), encoding="utf-8")


def save_joblib_run(model_dir, models, layout, report, features):
    model_dir.mkdir(parents=True, exist_ok=True)
    for target, model in models.items():
        joblib.dump(model, model_dir / f"{target}.joblib")
    save_run_metadata(model_dir, layout, report)
    write_estimates(model_dir / "estimates.csv", models, features)


def predict_joblib_run(args):
    features = prepare_prediction(args)
    models = {target: joblib.load(args.model_dir / f"{target}.joblib") for target in TARGETS}
    write_estimates(args.output, models, features)


def estimator_cli(description, data_file, validation_type, data_split, model_dir, predictions_file):
    parser = argparse.ArgumentParser(description=description)
    commands = parser.add_subparsers(dest="command", required=True)
    fit = commands.add_parser("train", help="fit the model and estimate the input rows")
    infer = commands.add_parser("predict", help="estimate rows using a saved model")
    for command in (fit, infer):
        command.add_argument(
            "matfile", nargs="?", type=Path, default=data_file,
            help="one MAT-file path, defaults to DATA_FILE in this script",
        )
        command.add_argument("--variable", default="interpoled_data", help="MAT-file variable name")
        command.add_argument("--model-dir", type=Path, default=Path(model_dir))
        command.add_argument("--matlab", help="MATLAB executable path")
        command.add_argument("--plain-mat", action="store_true", help="read a v5-style numeric struct using SciPy")
    add_validation_options(fit, validation_type, data_split)
    infer.add_argument("--output", type=Path, default=Path(predictions_file))
    return parser, fit, infer


def run_cli(parser, train, predict):
    args = parser.parse_args(sys.argv[1:] or ["train"])
    try:
        operation = train if args.command == "train" else predict
        operation(args)
    except (OSError, RuntimeError, ValueError, subprocess.CalledProcessError) as error:
        parser.exit(status=1, message=f"Error: {error}\n")
