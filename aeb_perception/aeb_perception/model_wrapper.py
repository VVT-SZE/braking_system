"""Egyszer betöltött MMDetection3D modell wrapper.

Ugyanazt a configot és checkpointot használja, mint a
notebooks/centerpoint_devcontainer.ipynb, csak itt élő, memóriában lévő
numpy-tömb a bemenet, nem egy fájlból betöltött dataset-rekord.
"""
from typing import List

import numpy as np
from mmengine.config import Config
from mmengine.registry import init_default_scope
from mmdet3d.apis import inference_detector, init_model


class CenterPointWrapper:
    """A notebook 19. cellájában bemutatott inferencia-mintát csomagolja be.

    A `init_model`-nek adott configban a pipeline-t explicit úgy írjuk át,
    hogy ne fájlból próbáljon pontokat betölteni (az eredeti config a
    dataset info-fájlra épít) - mi már előfeldolgozott numpy-tömböt adunk.
    """

    def __init__(self, model_config_path: str, checkpoint_path: str, device: str = "cuda:0"):
        base_cfg = Config.fromfile(model_config_path)
        init_default_scope(base_cfg.get("default_scope", "mmdet3d"))

        inference_cfg = Config.fromfile(model_config_path)
        inference_cfg.test_dataloader.dataset.pipeline = [
            dict(type="LoadPointsFromFile", coord_type="LIDAR", load_dim=5, use_dim=5),
            dict(type="Pack3DDetInputs", keys=["points"]),
        ]

        self.model = init_model(inference_cfg, checkpoint_path, device=device)
        self.classes: List[str] = self.model.dataset_meta["classes"]

    def infer(self, points: np.ndarray):
        """points: (N, 5) float32 tömb, oszlopok [x, y, z, intensity, dt].

        Visszaadja a `pred_instances_3d`-t (ugyanaz a típus, mint a
        notebookban: .bboxes_3d, .scores_3d, .labels_3d).
        """
        output = inference_detector(self.model, points)
        result = output[0] if isinstance(output, tuple) else output
        return result.pred_instances_3d
