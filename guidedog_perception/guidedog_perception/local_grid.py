# SPDX-License-Identifier: Apache-2.0
"""Bounded rolling evidence grid for observed flat ground and positive obstacles."""

import math

import numpy as np


class LocalGrid:
    """Keep odom-aligned cells, explicit unknowns and independent wall-time expiry."""

    def __init__(self, resolution=0.1, size=8.0, free_ttl=2.0, occupied_ttl=5.0,
                 clear_scans=3):
        """Allocate fixed storage; evidence scores are not calibrated probabilities."""
        if (not all(math.isfinite(v) and v > 0 for v in
                    (resolution, size, free_ttl, occupied_ttl))
                or occupied_ttl < free_ttl or not isinstance(clear_scans, int)
                or not 2 <= clear_scans <= 100):
            raise ValueError('invalid_grid_parameters')
        self.n = int(round(size / resolution))
        if not 8 <= self.n <= 200 or abs(self.n*resolution - size) > 1e-6:
            raise ValueError('grid_size_must_be_an_integer_number_of_8_to_200_cells')
        self.resolution, self.size = resolution, size
        self.free_ttl, self.occupied_ttl = free_ttl, occupied_ttl
        self.clear_scans = clear_scans
        self.arrays = {
            'hit_at': np.full((self.n, self.n), -np.inf),
            'floor_at': np.full((self.n, self.n), -np.inf),
            'floor_z': np.full((self.n, self.n), np.nan, dtype=np.float32),
            'floor_rms': np.full((self.n, self.n), np.nan, dtype=np.float32),
            'clear_streak': np.zeros((self.n, self.n), dtype=np.uint16),
            'hit_scans': np.zeros((self.n, self.n), dtype=np.uint16),
        }
        self.origin_cells = None
        self.center = None
        self.last_stamp = None
        self.rolls = 0
        self.resets = 0

    def reset(self):
        """Discard every spatial association on a clock reset or localization jump."""
        for name, array in self.arrays.items():
            array.fill(-np.inf if name.endswith('_at') else
                       np.nan if name.startswith('floor_') else 0)
        self.origin_cells = self.center = self.last_stamp = None
        self.resets += 1

    def recenter(self, center):
        """Shift by whole cells without rotating or wrapping old world observations."""
        center = np.asarray(center, dtype=float)
        if center.shape != (2,) or not np.isfinite(center).all():
            raise ValueError('invalid_grid_center')
        if self.center is not None and np.linalg.norm(center - self.center) > 1.0:
            self.reset()
        origin = np.floor(center / self.resolution).astype(np.int64) - self.n//2
        if self.origin_cells is not None:
            dx, dy = origin - self.origin_cells
            if dx or dy:
                for name, array in self.arrays.items():
                    fill = (-np.inf if name.endswith('_at') else
                            np.nan if name.startswith('floor_') else 0)
                    if abs(dx) >= self.n or abs(dy) >= self.n:
                        array.fill(fill)
                        continue
                    array[:] = np.roll(array, (-dy, -dx), axis=(0, 1))
                    if dx > 0:
                        array[:, -dx:] = fill
                    elif dx < 0:
                        array[:, :-dx] = fill
                    if dy > 0:
                        array[-dy:, :] = fill
                    elif dy < 0:
                        array[:-dy, :] = fill
                self.rolls += 1
        self.origin_cells, self.center = origin, center

    @property
    def origin(self):
        """Return the lower-left cell corner in odom metres."""
        return self.origin_cells*self.resolution if self.origin_cells is not None else np.zeros(2)

    @property
    def memory_bytes(self):
        """Account for all persistent cell arrays."""
        return sum(a.nbytes for a in self.arrays.values())

    def observe(self, local, odom, plane, center, stamp, now, *, body_mask=True,
                excluded_odom=None):
        """Accumulate only unique stamped observations; never ray-clear absent ground."""
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return False
        local, odom, plane = np.asarray(local), np.asarray(odom), np.asarray(plane)
        if (local.ndim != 2 or local.shape[1] != 3 or odom.shape != local.shape
                or plane.shape != (3,) or not np.isfinite(plane).all()
                or not math.isfinite(now) or stamp < 0):
            raise ValueError('invalid_grid_observation')
        self.recenter(center)
        valid = np.isfinite(local).all(axis=1) & np.isfinite(odom).all(axis=1)
        local, odom = local[valid], odom[valid]
        # The body/blind region provides no observed floor evidence.
        outside_body = ((np.abs(local[:, 0]) > 0.45) | (np.abs(local[:, 1]) > 0.40)
                        if body_mask else np.ones(len(local), dtype=bool))
        height = local[:, 2] - (local[:, :2] @ plane[:2] + plane[2])
        cells = np.floor(odom[:, :2] / self.resolution).astype(np.int64) - self.origin_cells
        inside = outside_body & ((cells >= 0) & (cells < self.n)).all(axis=1)
        cells, height, odom = cells[inside], height[inside], odom[inside]
        indices = cells[:, 1]*self.n + cells[:, 0]
        ground = np.abs(height) <= 0.025
        obstacle = (height >= 0.08) & (height <= 1.5)
        count = self.n*self.n
        floors = np.bincount(indices[ground], minlength=count).reshape(self.n, self.n)
        hits = np.bincount(indices[obstacle], minlength=count).reshape(self.n, self.n) >= 2
        nonfloor = np.bincount(indices[~ground], minlength=count).reshape(self.n, self.n) > 0
        if excluded_odom is not None:
            excluded = np.asarray(excluded_odom)
            if excluded.ndim != 2 or excluded.shape[1] != 3:
                raise ValueError('invalid_excluded_points')
            excluded = excluded[np.isfinite(excluded).all(axis=1)]
            cells = (np.floor(excluded[:, :2]/self.resolution).astype(np.int64)
                     - self.origin_cells)
            cells = cells[((cells >= 0) & (cells < self.n)).all(axis=1)]
            # A removed body/boundary hit can withhold a cell, never create free floor
            # or clear a previously observed obstacle in that same projected cell.
            nonfloor[cells[:, 1], cells[:, 0]] = True
        free = (floors >= 3) & ~nonfloor
        a = self.arrays
        a['clear_streak'][now - a['floor_at'] > self.free_ttl] = 0
        a['hit_scans'][now - a['hit_at'] > self.occupied_ttl] = 0
        a['clear_streak'][~free] = 0
        a['clear_streak'][free] = np.minimum(a['clear_streak'][free] + 1, self.clear_scans)
        a['hit_scans'][hits] = np.minimum(a['hit_scans'][hits] + 1, self.clear_scans)
        a['hit_at'][hits] = now
        # Sparse obstacles, low steps or unsupported heights cannot establish free floor.
        a['floor_at'][nonfloor] = -np.inf
        cleared = free & (a['clear_streak'] >= self.clear_scans)
        a['hit_at'][cleared] = -np.inf
        a['hit_scans'][cleared] = 0
        a['floor_at'][free] = now
        sums = np.bincount(indices[ground], weights=odom[ground, 2], minlength=count)
        squares = np.bincount(indices[ground], weights=odom[ground, 2]**2, minlength=count)
        means = sums.reshape(self.n, self.n) / np.maximum(floors, 1)
        rms = np.sqrt(np.maximum(0, squares.reshape(self.n, self.n) / np.maximum(floors, 1)
                                 - means**2))
        a['floor_z'][free] = means[free]
        a['floor_rms'][free] = rms[free]
        self.last_stamp = stamp
        return True

    def snapshot(self, now):
        """Expire observations to unknown and expose age, support and evidence layers."""
        a = self.arrays
        occupied = now - a['hit_at'] <= self.occupied_ttl
        free = (now - a['floor_at'] <= self.free_ttl) & ~occupied
        data = np.full((self.n, self.n), -1, dtype=np.int8)
        data[free], data[occupied] = 0, 100
        age = np.full(data.shape, np.nan, dtype=np.float32)
        age[occupied] = now - a['hit_at'][occupied]
        age[free] = now - a['floor_at'][free]
        evidence = np.full(data.shape, np.nan, dtype=np.float32)
        evidence[occupied] = a['hit_scans'][occupied] / self.clear_scans
        evidence[free] = a['clear_streak'][free] / self.clear_scans
        support_valid = now - a['floor_at'] <= self.free_ttl
        floor = np.where(support_valid, a['floor_z'], np.nan)
        rms = np.where(support_valid, a['floor_rms'], np.nan)
        return {'occupancy': data, 'age_sec': age, 'evidence_score': evidence,
                'ground_z_odom_m': floor, 'ground_rms_m': rms}
