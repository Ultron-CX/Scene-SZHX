"""Local SegFormer evaluation backend. Never uploads inputs or reads encrypted files."""
import base64
import hashlib
import io
import json
import os
import threading
import time
from pathlib import Path

import numpy as np
from PIL import Image

from scenesem_config import MODELS, POSES
MODEL = MODELS / 'segformer-b0-finetuned-ade-512-512'
MODEL_ID = 'nvidia/segformer-b0-finetuned-ade-512-512'
# Explicit ADE20K-to-workbench mapping. Unmapped labels remain unknown.
LABELS = {
    8: ('window', '窗'), 10: ('cabinet', '柜子'), 14: ('door', '门'),
    15: ('table', '桌子'), 17: ('plant', '绿植'), 19: ('chair', '椅子'),
    22: ('picture', '画 / 展示画面'), 23: ('sofa', '沙发'), 24: ('shelf', '架子'),
    33: ('table', '书桌'), 36: ('lamp', '灯具'), 43: ('sign', '标识牌'),
    45: ('counter', '柜台'), 50: ('device', '冰箱'), 58: ('door', '纱门'),
    62: ('shelf', '书架'), 64: ('table', '茶几'), 67: ('book', '书'),
    74: ('computer', '计算机'), 89: ('screen', '电视'), 98: ('bottle', '瓶子'),
    124: ('device', '微波炉'), 130: ('screen', '屏幕'),
    141: ('screen', 'CRT 屏幕'), 143: ('screen', '显示器'),
}


class ObjectSemantics:
    def __init__(self):
        self.lock = threading.Lock()
        self.model = None

    def load(self):
        os.environ['HF_HUB_OFFLINE'] = '1'
        os.environ['HF_HUB_DISABLE_TELEMETRY'] = '1'
        from transformers import SegformerImageProcessor, SegformerForSemanticSegmentation
        import torch
        torch.set_num_threads(min(4, os.cpu_count() or 1))
        self.processor = SegformerImageProcessor.from_pretrained(str(MODEL), local_files_only=True)
        self.model = SegformerForSemanticSegmentation.from_pretrained(str(MODEL), local_files_only=True, use_safetensors=True).eval()
        self.model_hash = hashlib.sha256((MODEL / 'model.safetensors').read_bytes()).hexdigest()

    def infer(self, image, confidence=.6):
        if not .35 <= confidence <= .9:
            raise ValueError('Confidence must be in [0.35, 0.9]')
        if image.width > 768 or image.height > 768 or min(image.size) < 64:
            raise ValueError('Image must be between 64 and 768 pixels per side')
        if not self.lock.acquire(blocking=False):
            raise RuntimeError('模型正在处理另一张图，请稍后重试')
        try:
            started = time.perf_counter()
            if self.model is None:
                self.load()
            import torch
            from scipy import ndimage
            with torch.inference_mode():
                inputs = self.processor(images=image.convert('RGB'), return_tensors='pt')
                logits = self.model(**inputs).logits
                logits = torch.nn.functional.interpolate(logits, size=(image.height, image.width), mode='bilinear', align_corners=False)[0]
                scores, classes = torch.softmax(logits, dim=0).max(dim=0)
                labels = classes.numpy()
                scores = scores.numpy()
            components = []
            # Connected visible regions, NOT authoritative object instances.
            for label_id, (category, display) in LABELS.items():
                mask = (labels == label_id) & (scores >= confidence)
                cc, count = ndimage.label(mask)
                sizes = np.bincount(cc.ravel())
                for index in np.flatnonzero(sizes[1:] >= 24) + 1:
                    yy, xx = np.nonzero(cc == index)
                    if len(xx) > image.width * image.height * .65:
                        continue
                    runs = []
                    for row in np.unique(yy):
                        cols = xx[yy == row]
                        cuts = np.flatnonzero(np.diff(cols) > 1) + 1
                        for part in np.split(cols, cuts):
                            runs.append([int(row), int(part[0]), int(part[-1] + 1)])
                    components.append(dict(category=category, label=display, label_id=label_id,
                        pixels=len(xx), score=float(scores[yy, xx].mean()), runs=runs))
            components.sort(key=lambda item: item['pixels'], reverse=True)
            return dict(model=MODEL_ID, model_sha256=self.model_hash, license_scope='research_evaluation_only',
                        width=image.width, height=image.height, threshold=confidence,
                        seconds=round(time.perf_counter() - started, 3), components=components[:100],
                        discarded_component_count=max(0, len(components) - 100),
                        qualification=False, real_robot_commands=False)
        finally:
            self.lock.release()


def decode_request(data, scene):
    if data.get('scene_id') != scene['scene_id'] or data.get('source_hashes') != scene['source_hashes']:
        raise ValueError('场景批次不匹配')
    raw = base64.b64decode(data['image'], validate=True)
    if len(raw) > 3 * 1024 * 1024:
        raise ValueError('图像过大')
    image = Image.open(io.BytesIO(raw))
    if max(image.size) > 1024 or min(image.size) < 64:
        raise ValueError('图像尺寸超出限制')
    image.load()
    return image, float(data.get('confidence', .6)), hashlib.sha256(raw).hexdigest()


def tour_positions(scene):
    """Use readable export positions only; no camera intrinsics or encrypted SfM files."""
    pose_file = POSES
    matrix = np.asarray(scene['T_lcc_from_las'])
    positions = []
    for line in pose_file.read_text(encoding='utf-8-sig').splitlines():
        fields = line.split()
        if not fields or fields[0].startswith('#'):
            continue
        xyz = np.asarray([float(x) for x in fields[2:5]]) @ matrix[:3, :3].T + matrix[:3, 3]
        if np.isfinite(xyz).all() and all(np.linalg.norm(xyz - p) > 3.0 for p in positions):
            positions.append(xyz)
    if len(positions) > 12:
        positions = [positions[i] for i in np.linspace(0, len(positions)-1, 12).astype(int)]
    return dict(scene_id=scene['scene_id'], source_hashes=scene['source_hashes'], positions=[p.tolist() for p in positions],
                pose_source_sha256=hashlib.sha256(pose_file.read_bytes()).hexdigest(),
                coverage='sampled_scan_positions_not_full_scene_coverage', alignment='trajectory_candidate_not_surface_certified')
