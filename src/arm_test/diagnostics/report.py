"""Result model + reporting for diagnostics.

Every check returns a `CheckResult`. A `Report` collects them, prints a
human-readable table (via rich), and can dump JSON for record-keeping so you
build a maintenance history per arm/serial.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field, asdict
from enum import Enum
from typing import Any, Optional


class Status(str, Enum):
    PASS = "PASS"
    WARN = "WARN"
    FAIL = "FAIL"
    SKIP = "SKIP"


@dataclass
class CheckResult:
    name: str                       # e.g. "J3 motor feedback"
    status: Status
    detail: str = ""                # short human explanation
    data: dict[str, Any] = field(default_factory=dict)  # raw numbers for records

    @property
    def ok(self) -> bool:
        return self.status in (Status.PASS, Status.SKIP)


@dataclass
class Report:
    title: str = "YAM Pro diagnostics"
    arm_serial: Optional[str] = None
    timestamp: Optional[str] = None  # caller stamps this (ISO string)
    results: list[CheckResult] = field(default_factory=list)

    def add(self, result: CheckResult) -> CheckResult:
        self.results.append(result)
        return result

    @property
    def failed(self) -> list[CheckResult]:
        return [r for r in self.results if r.status == Status.FAIL]

    @property
    def worst(self) -> Status:
        order = [Status.FAIL, Status.WARN, Status.PASS, Status.SKIP]
        for s in order:
            if any(r.status == s for r in self.results):
                return s
        return Status.SKIP

    # ---- output -----------------------------------------------------------
    def to_dict(self) -> dict:
        d = asdict(self)
        d["worst"] = self.worst.value
        return d

    def to_json(self, path: Optional[str] = None, indent: int = 2) -> str:
        text = json.dumps(self.to_dict(), indent=indent, default=str)
        if path:
            with open(path, "w", encoding="utf-8") as f:
                f.write(text)
        return text

    def print(self) -> None:
        """Pretty table if rich is available, else plain text."""
        try:
            from rich.console import Console
            from rich.table import Table

            style = {
                Status.PASS: "green",
                Status.WARN: "yellow",
                Status.FAIL: "bold red",
                Status.SKIP: "dim",
            }
            console = Console()
            table = Table(title=self.title, show_lines=False)
            table.add_column("Check")
            table.add_column("Status")
            table.add_column("Detail")
            for r in self.results:
                table.add_row(r.name, f"[{style[r.status]}]{r.status.value}[/]", r.detail)
            console.print(table)
            console.print(f"Overall: [{style[self.worst]}]{self.worst.value}[/]")
        except ImportError:
            print(f"\n== {self.title} ==")
            for r in self.results:
                print(f"  [{r.status.value:4}] {r.name}: {r.detail}")
            print(f"  Overall: {self.worst.value}")
