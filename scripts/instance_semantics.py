"""Offline COCO instance candidates, with optional overlapping local detail crops.

Masks describe visible pixels, not physical objects or navigation truth. The old
SegFormer engine stays in object_semantics.py for reproducible comparisons.
"""
import hashlib
import os
import threading
import time
from pathlib import Path

import numpy as np

from scenesem_config import MODELS
MODEL = MODELS / 'mask2former-swin-tiny-coco-instance'
MODEL_ID = 'facebook/mask2former-swin-tiny-coco-instance'
REVISION = '22c4a2f15dc88149b8b8d9f4d42c54431fbd66f6'
# Deliberately no inferred cabinets, doors, windows, signs or button functions.
LABELS = {
    'chair': ('chair', '椅子'), 'couch': ('sofa', '沙发'),
    'potted plant': ('plant', '绿植'), 'dining table': ('table', '桌子'),
    'tv': ('screen', '显示器 / 电视'), 'laptop': ('computer', '笔记本电脑'),
    'keyboard': ('keyboard', '键盘'), 'mouse': ('mouse', '鼠标'),
    'bottle': ('bottle', '瓶子'), 'cup': ('cup', '杯子'), 'book': ('book', '书'),
    'microwave': ('device', '微波炉'), 'refrigerator': ('device', '冰箱'),
}
# This checkpoint uses older COCO spellings in config.id2label.
LABELS.update({alias: LABELS[name] for alias, name in {
    'sofa': 'couch', 'pottedplant': 'potted plant', 'diningtable': 'dining table',
    'tvmonitor': 'tv',
}.items()})


def detail_crops(width, height, detail, target_roi=None):
    if detail not in ('standard', 'detail', 'target'):
        raise ValueError('识别模式须为 standard、detail 或 target')
    boxes = [(0, 0, width, height)]
    if detail == 'detail':
        # 2x2 tiles, 24% overlap, keeping native pixels (not downsampled first).
        cw, ch = round(width * .62), round(height * .62)
        boxes += [(x, y, x+cw, y+ch) for y in (0, height-ch) for x in (0, width-cw)]
    if detail == 'target':
        if (not isinstance(target_roi, list) or len(target_roi) != 4 or
                any(type(v) is not int for v in target_roi)):
            raise ValueError('目标窗口须为四个整数像素坐标')
        x0, y0, x1, y1 = target_roi
        if not (0 <= x0 < x1 <= width and 0 <= y0 < y1 <= height and
                x1-x0 >= 64 and y1-y0 >= 64):
            raise ValueError('目标窗口超出图像范围或小于 64 像素')
        if tuple(target_roi) != boxes[0]:
            boxes.append(tuple(target_roi))
    return boxes


def mask_runs(mask):
    runs = []
    for y in np.flatnonzero(mask.any(axis=1)):
        cols = np.flatnonzero(mask[y])
        for part in np.split(cols, np.flatnonzero(np.diff(cols) > 1)+1):
            runs.append([int(y), int(part[0]), int(part[-1]+1)])
    return runs


def deduplicate(candidates):
    """Keep complete masks over crop-edge fragments. Never union bounding boxes."""
    kept = []
    for c in sorted(candidates, key=lambda c: (not c['crop_truncated'], c['score']), reverse=True):
        duplicate = False
        for old in kept:
            if old['category'] != c['category']:
                continue
            intersection = int(np.count_nonzero(old['mask'] & c['mask']))
            union = old['pixels'] + c['pixels'] - intersection
            if intersection / max(1, union) >= .5 or intersection / min(old['pixels'], c['pixels']) >= .85:
                old['support_crops'] = sorted(set(old['support_crops'] + c['support_crops']))
                duplicate = True
                break
        # Crop-edge-only discoveries are too easily truncated; don't promote them.
        if not duplicate and not c['crop_truncated']:
            kept.append(c)
    return kept[:100]


class InstanceSemantics:
    def __init__(self):
        self.lock = threading.Lock()
        self.model = None

    def load(self):
        os.environ['HF_HUB_OFFLINE'] = '1'
        os.environ['HF_HUB_DISABLE_TELEMETRY'] = '1'
        import torch
        from transformers import Mask2FormerImageProcessor, Mask2FormerForUniversalSegmentation
        torch.set_num_threads(min(4, os.cpu_count() or 1))
        self.processor = Mask2FormerImageProcessor.from_pretrained(str(MODEL), local_files_only=True)
        self.model = Mask2FormerForUniversalSegmentation.from_pretrained(str(MODEL), local_files_only=True, use_safetensors=True).eval()
        self.model_hash = hashlib.sha256((MODEL / 'model.safetensors').read_bytes()).hexdigest()

    def _predict(self, image, confidence):
        import torch
        import torch.nn.functional as F
        with torch.inference_mode():
            inputs = self.processor(images=image, return_tensors='pt', size={'shortest_edge': 640, 'longest_edge': 1024})
            output = self.model(**inputs)
            # One class per object query, not semantic connected-component blobs.
            scores, labels = output.class_queries_logits[0].softmax(-1)[:, :-1].max(-1)
            ph, pw = inputs['pixel_values'].shape[-2:]
            valid = inputs['pixel_mask'][0].bool()
            rh, rw = int(valid.any(1).sum()), int(valid.any(0).sum())
            results = []
            for q in torch.where(scores >= confidence)[0].tolist():
                name = self.model.config.id2label[int(labels[q])]
                if name not in LABELS:
                    continue
                # Remove processor padding before mapping back to image pixels.
                logits = F.interpolate(output.masks_queries_logits[0, q][None, None], size=(ph, pw), mode='bilinear', align_corners=False)[0, 0][:rh, :rw]
                support = logits > 0
                if not support.any():
                    continue
                score = float(scores[q] * logits.sigmoid()[support].mean())
                if score < confidence:
                    continue
                mask = F.interpolate(logits[None, None], size=(image.height, image.width), mode='bilinear', align_corners=False)[0, 0].numpy() > 0
                if mask.sum() < 24:
                    continue
                results.append(dict(mask=mask, score=score, label_id=int(labels[q]), query_id=q,
                                    category=LABELS[name][0], label=LABELS[name][1], model_label=name))
            return results

    def infer(self, image, confidence=.6, detail='standard', target_roi=None):
        if not .35 <= confidence <= .9:
            raise ValueError('Confidence must be in [0.35, 0.9]')
        if max(image.size) > 1024 or min(image.size) < 64:
            raise ValueError('Image must be between 64 and 1024 pixels per side')
        crops = detail_crops(*image.size, detail, target_roi)
        if not self.lock.acquire(blocking=False):
            raise RuntimeError('模型正在处理另一张图，请稍后重试')
        try:
            started = time.perf_counter()
            if self.model is None:
                self.load()
            candidates = []
            for crop_id, (x0, y0, x1, y1) in enumerate(crops):
                for item in self._predict(image.crop((x0, y0, x1, y1)).convert('RGB'), confidence):
                    local = item['mask']
                    # Only internal crop edges count; the original frame edge is visible evidence.
                    truncated = ((x0 > 0 and local[:, :2].any()) or (y0 > 0 and local[:2, :].any()) or
                                 (x1 < image.width and local[:, -2:].any()) or (y1 < image.height and local[-2:, :].any()))
                    mask = np.zeros((image.height, image.width), dtype=bool)
                    mask[y0:y1, x0:x1] = local
                    candidates.append(dict(item, mask=mask, pixels=int(mask.sum()), crop_truncated=bool(truncated),
                                           crop_id=crop_id, support_crops=[crop_id]))
            # Target refinement needs both full-frame and ROI evidence. Ordinary
            # containment NMS can discard a more complete ROI chair in favour of
            # a full-frame seat-only mask. The client checks target anchors and
            # cross-crop agreement before proposing any 3D union.
            kept = ([c for c in candidates if not c['crop_truncated']][:100]
                    if detail == 'target' else deduplicate(candidates))
            components = []
            for i, item in enumerate(kept):
                mask = item.pop('mask')
                components.append(dict(item, instance_id=i, runs=mask_runs(mask)))
            return dict(model=MODEL_ID, model_revision=REVISION, model_sha256=self.model_hash,
                        license_scope='evaluation_pending_weights_license_review', backend='instance', detail=detail,
                        method='mask2former_rgb_gs_depth_instance_support_v2', cell_m=.04,
                        width=image.width, height=image.height, threshold=confidence, components=components,
                        raw_mask_count=len(candidates), crop_count=len(crops),
                        target_roi=target_roi if detail == 'target' else None,
                        seconds=round(time.perf_counter()-started, 3), qualification=False, real_robot_commands=False)
        finally:
            self.lock.release()
