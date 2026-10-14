# ================================================================
# terrain_preview.py
# 설명: 지형 층과 costmap을 단위·범례가 있는 정적 분석 그림으로 저장한다.
# 사용법: from experiments.handheld_mapping.terrain_preview import render
# ================================================================
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm, Normalize
from matplotlib.cm import ScalarMappable
from matplotlib.patches import Patch
import numpy as np

from experiments.handheld_mapping.terrain import GridSpec


def render(layers: dict, cost: np.ndarray, spec: GridSpec, path: Path) -> None:
    """높이·경사·거칠기·비용을 동일한 XY 범위에 표시한다."""
    fig, axes = plt.subplots(2, 2, figsize=(11, 9), constrained_layout=True)
    extent = (spec.bounds[0], spec.bounds[0]+cost.shape[1]*spec.resolution,
              spec.bounds[2], spec.bounds[2]+cost.shape[0]*spec.resolution)
    for axis, key, title, unit in zip(axes.flat, ('height_m', 'slope_deg', 'roughness_m'),
                                    ('Elevation', 'Slope', 'Local plane residual roughness'), ('m', 'deg', 'm')):
        data = np.where(layers['observed'], layers[key], np.nan)
        artist = axis.imshow(data, origin='lower', extent=extent, cmap='viridis')
        axis.set_title(title)
        fig.colorbar(artist, ax=axis, label=unit)
    palette = list(plt.get_cmap('YlOrRd')(np.linspace(.1, .85, 253)))+[(1, 0, 0, 1), (.5, .5, .5, 1)]
    display = np.where(cost == 255, 254, np.where(cost == 254, 253, cost))
    axes[1, 1].imshow(display, origin='lower', extent=extent, cmap=ListedColormap(palette),
                      norm=BoundaryNorm(np.arange(256)-.5, 255))
    axes[1, 1].set_title('Cost: red blocked / grey unknown')
    fig.colorbar(ScalarMappable(norm=Normalize(0, 252), cmap=ListedColormap(palette[:253])),
                 ax=axes[1, 1], label='valid traversal cost', ticks=[0, 126, 252])
    axes[1, 1].legend(handles=[Patch(color='red', label='Blocked'), Patch(color='grey', label='Unknown')],
                       loc='upper right', fontsize=8)
    for axis in axes.flat:
        axis.set_xlabel('x [m]')
        axis.set_ylabel('y [m]')
        axis.set_aspect('equal')
    fig.suptitle('Terrain layers — prototype thresholds, accuracy not validated on real camera')
    fig.savefig(path, dpi=140)
    plt.close(fig)
