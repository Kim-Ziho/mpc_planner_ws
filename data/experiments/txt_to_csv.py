#!/usr/bin/env python3
"""Convert RosTools DataSaver recordings (.txt) to CSV."""
from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Dict, List, Tuple

DatasetRows = Dict[str, List[List[float]]]
DatasetDims = Dict[str, int]


def parse_dataset_file(path: Path) -> tuple[DatasetRows, DatasetDims, List[str]]:
    """Parse a DataSaver text file into per-dataset rows and dimensions."""
    rows: DatasetRows = {}
    dims: DatasetDims = {}
    order: List[str] = []
    with path.open() as infile:
        while True:
            header = infile.readline()
            if not header:
                break
            header = header.strip()
            if not header:
                continue
            if header == "-1":
                break

            if ":" not in header:
                raise ValueError(f"Malformed header line: {header!r}")
            name, rest = header.split(":", 1)
            parts = rest.strip().split()
            if len(parts) != 2:
                raise ValueError(f"Malformed header fields: {header!r}")

            try:
                num_columns = int(parts[0])
                count = int(parts[1])
            except ValueError as exc:
                raise ValueError(f"Invalid dimension/count in {header!r}") from exc

            dataset_name = name.strip()
            if dataset_name not in rows:
                order.append(dataset_name)
                rows[dataset_name] = []
            dims[dataset_name] = num_columns
            bucket = rows[dataset_name]

            for idx in range(count):
                data_line = infile.readline()
                if not data_line:
                    raise ValueError(
                        f"Unexpected EOF while reading values for dataset '{name.strip()}'"
                    )
                values = [float(token) for token in data_line.strip().split()]
                if num_columns > 0 and len(values) != num_columns:
                    raise ValueError(
                        f"Expected {num_columns} values for dataset '{name.strip()}', "
                        f"got {len(values)} on sample {idx}"
                    )
                bucket.append(values)
    return rows, dims, order


def write_csv(rows: DatasetRows, dims: DatasetDims, order: List[str], output: Path) -> None:
    """Write datasets as columns with a shared index column."""
    dataset_names = order
    header = ["index"]
    column_slices: list[tuple[str, int]] = []
    for name in dataset_names:
        dim = dims.get(name, 1)
        if dim <= 1:
            header.append(name)
        else:
            header.extend(f"{name}_{i}" for i in range(dim))
        column_slices.append((name, dim))

    max_rows = max((len(values) for values in rows.values()), default=0)

    with output.open("w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(header)
        for idx in range(max_rows):
            record: List[str | float] = [idx]
            for name, dim in column_slices:
                dataset = rows[name]
                if idx < len(dataset):
                    values = dataset[idx]
                    if dim == 1:
                        record.append(values[0])
                    else:
                        record.extend(values)
                else:
                    if dim == 1:
                        record.append("")
                    else:
                        record.extend("" for _ in range(dim))
            writer.writerow(record)


def convert(input_path: Path, output_path: Path | None) -> Path:
    """Convert a single DataSaver file to CSV."""
    if output_path is None:
        output_path = input_path.with_suffix(".csv")

    rows, dims, order = parse_dataset_file(input_path)
    write_csv(rows, dims, order, output_path)
    return output_path


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Convert RosTools DataSaver .txt recordings to CSV."
    )
    parser.add_argument("input", type=Path, help="Path to the .txt recording file")
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        help="Destination CSV path (defaults to input path with .csv extension)",
    )
    args = parser.parse_args()

    output = convert(args.input, args.output)
    print(f"Wrote CSV to {output}")


if __name__ == "__main__":
    main()
