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

UGYANEZ IGAZ A MOZGÓ CÉLOBJEKTUMRA IS, és ez a gyakorlatban súlyosabb, mint
az ego-mozgás: ha egy előttünk haladó/mozgó autó pontjai 5 sweepen (~0.5s)
át egyszerűen egymásra kerülnek, a pontjai NEM egy helyen halmozódnak, mert
az autó közben elmozdult - a modell egy, a valós méreténél jóval nagyobb
("szétkent") dobozt lát, és ez könnyen átviszi az osztályozást 'car'-ról
'truck'/'bus'-ra (mért tény, lásd config/params.yaml `num_sweeps`
megjegyzése: 4.5m hosszú doboz -> 11-13m, amint a célautó elkezd mozogni).
Ezt NEM oldja meg az odom-alapú ego-motion-kompenzáció (ami csak az EGO
saját mozgására korrigálna) - ehhez a célobjektum SAJÁT sebességét kellene
ismerni/becsülni sweepek közt, ami lényegében egy tracker. Amíg ilyen
nincs, a kompromisszum az alacsony `num_sweeps` (lásd params.yaml).

2026-10-29: point_cloud_range-vágás hozzáadva (lásd config/params.yaml
`point_cloud_range` megjegyzése) - a valódi luminar_pcl 360°-ban, akár
200+ méterig lát, de a checkpoint csak a sajátjánál (kb. ±51.2 m) kijjebb
eső pontokkal semmit nem tud kezdeni - a voxelizáló réteg ezeket amúgy is
csendben eldobta volna, csak a teljes pillar-pipeline-on már
feleslegesen átmentek (mért hatás: kb. 4x-10x inferencia-lassulás, lásd
README "2026-10-29 javítások"). Itt vágjuk le, a sweep-pufferbe kerülés
ELŐTT, hogy a puffer memóriája és a modellbemenet is kisebb legyen.
"""
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Tuple

import numpy as np


@dataclass
class TimedSweep:
    points_base_link: np.ndarray  # (N, 4): x, y, z, intensity - base_link frame-ben
    stamp_sec: float


class SweepBuffer:
    def __init__(
        self,
        max_sweeps: int = 10,
        point_cloud_range: Optional[Tuple[float, float, float, float, float, float]] = None,
    ):
        """point_cloud_range: (x_min, y_min, z_min, x_max, y_max, z_max).

        Ha meg van adva, minden push()-olt sweep pontjai ezen a tartományon
        kívül azonnal el lesznek dobva (a modell point_cloud_range-ével kell
        megegyezzen, lásd config/params.yaml). Ha None, nincs vágás (a régi
        viselkedés)."""
        self._buffer: Deque[TimedSweep] = deque(maxlen=max_sweeps)
        self._point_cloud_range = point_cloud_range

    def push(self, sweep: TimedSweep) -> None:
        if self._point_cloud_range is not None:
            sweep.points_base_link = self._crop_to_range(
                sweep.points_base_link, self._point_cloud_range
            )
        self._buffer.append(sweep)

    @staticmethod
    def _crop_to_range(points: np.ndarray, pc_range) -> np.ndarray:
        x_min, y_min, z_min, x_max, y_max, z_max = pc_range
        x, y, z = points[:, 0], points[:, 1], points[:, 2]
        mask = (
            (x >= x_min) & (x <= x_max)
            & (y >= y_min) & (y <= y_max)
            & (z >= z_min) & (z <= z_max)
        )
        return points[mask]

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
