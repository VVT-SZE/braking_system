"""LiDAR pontfelhő előfeldolgozás: sweep-puffer, ODOM NÉLKÜL.

A letöltött CenterPoint checkpoint 10 egymást követő sweepre lett tanítva
(lásd spec.md 2. szekció, "Sweep-puffer vagy egysweepes konfig" nyitott
kérdés). Ez a modul pufferben tartja az utolsó N sweepet.

EZ A VERZIÓ NEM KOMPENZÁL EGO-MOZGÁSRA. Az eredeti terv a sweepeket egy
közös referencia-frame-be (a legutóbbi sweep ego-pózához) forgatta/eltolta
volna, mielőtt a modellnek adja - ehhez a LiDAR-sweephez IDŐBEN
LEGKÖZELEBBI odom-mintára lett volna szükség (timestamp-illesztés két
aszinkron topic között, amit még nem építettünk meg - lásd
perception_node.py docstringje). Amíg ez nincs megoldva, a sweepek
EGYSZERŰEN egymás mellé kerülnek, mindegyik a saját (szenzor->base_link
statikus TF-fel már transzformált) koordinátájában, csak egy `dt` oszloppal
megjelölve, hogy melyik sweep mennyire "régi" a legfrissebbhez képest.

KÖVETKEZMÉNY, amit tudnod kell: ha az ego a sweep-ablakon (num_sweeps)
belül számottevően mozog/kanyarodik, a korábbi sweepek pontjai NEM lesznek
az aktuális ego-pózhoz igazítva - a többsweepes aggregáció "elmosódik".
Álló vagy lassan haladó ego esetén ez elhanyagolható; haladás/kanyarodás
közben ez az első dolog, amit vissza kell hozni, ha a detekciók
pontatlanok lesznek (lásd perception_node.py docstring: time-synchronizer
hozzáadása az odom visszakötéséhez).
"""
from collections import deque
from dataclasses import dataclass
from typing import Deque

import numpy as np


@dataclass
class TimedSweep:
    points_base_link: np.ndarray  # (N, 4): x, y, z, intensity - base_link frame-ben
    stamp_sec: float


class SweepBuffer:
    def __init__(self, max_sweeps: int = 10):
        self._buffer: Deque[TimedSweep] = deque(maxlen=max_sweeps)

    def push(self, sweep: TimedSweep) -> None:
        self._buffer.append(sweep)

    def is_ready(self) -> bool:
        return len(self._buffer) > 0

    def build_model_input(self) -> np.ndarray:
        """(N, 5) tömb [x, y, z, intensity, dt].

        ODOM NÉLKÜL: a pontok a saját (statikus TF-fel már base_link-be
        transzformált) koordinátájukban maradnak, nincs ego-mozgás szerinti
        forgatás/eltolás a sweepek között - csak a dt (másodperc, a
        legfrissebb sweephez képest mennyire régi) kerül rájuk."""
        if not self._buffer:
            raise RuntimeError("A sweep-puffer üres - még nem jött elég LiDAR-frame.")

        reference = self._buffer[-1]

        chunks = []
        for sweep in self._buffer:
            dt = reference.stamp_sec - sweep.stamp_sec

            n = sweep.points_base_link.shape[0]
            chunk = np.empty((n, 5), dtype=np.float32)
            chunk[:, 0:4] = sweep.points_base_link
            chunk[:, 4] = dt
            chunks.append(chunk)

        return np.concatenate(chunks, axis=0)
