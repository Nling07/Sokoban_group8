"""Run the solver on all bundled Sokoban maps and report steps and runtime."""

import argparse
import time

import solution
from sokoban import PROBLEMS
from validation import validate_solution


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--timebound",
        type=float,
        default=120,
        help="Solver time budget per map in seconds (default: 120)",
    )
    args = parser.parse_args()

    passed = 0
    total_solver_time = 0.0
    suite_start = time.perf_counter()

    for index, initial in enumerate(PROBLEMS, start=1):
        started = time.perf_counter()
        try:
            final = solution.solve(initial, timebound=args.timebound)
        except Exception as exc:
            elapsed = time.perf_counter() - started
            total_solver_time += elapsed
            print(
                f"Map {index:02}: ERROR | steps=- | time={elapsed:.3f}s "
                f"| {type(exc).__name__}: {exc}"
            )
            continue

        elapsed = time.perf_counter() - started
        total_solver_time += elapsed

        if final is False:
            print(
                f"Map {index:02}: FAIL | steps=- | time={elapsed:.3f}s "
                "| no solution within the solver budget"
            )
            continue

        valid, detail = validate_solution(initial, final)
        if valid:
            passed += 1
            print(
                f"Map {index:02}: PASS | steps={final.gval} "
                f"| time={elapsed:.3f}s"
            )
        else:
            print(
                f"Map {index:02}: INVALID | steps={final.gval} "
                f"| time={elapsed:.3f}s | {detail}"
            )

    wall_time = time.perf_counter() - suite_start
    print(
        f"\nResult: {passed}/{len(PROBLEMS)} maps solved with valid paths"
    )
    print(f"Total solver time: {total_solver_time:.3f}s")
    print(f"Total wall time:   {wall_time:.3f}s")


if __name__ == "__main__":
    main()
