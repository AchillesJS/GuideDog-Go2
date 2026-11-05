# SPDX-License-Identifier: Apache-2.0
"""Conservative visual/collision envelopes tied to a complete stamped robot pose."""

from dataclasses import dataclass
import hashlib
from importlib.resources import files
import json
from pathlib import Path
from urllib.parse import unquote, urlparse
import xml.etree.ElementTree as ET

from guidedog_perception.guard_geometry import transform_xyz
import numpy as np


def vector(text, count, default=None):
    """Reject malformed or nonfinite geometry instead of guessing dimensions."""
    value = np.array([float(x) for x in (text or default or '').split()])
    if value.shape != (count,) or not np.isfinite(value).all():
        raise ValueError('invalid_geometry_vector')
    return value


def rotation(rpy):
    """Return the URDF fixed-axis roll/pitch/yaw rotation into its parent frame."""
    r, p, y = rpy
    cr, cp, cy = np.cos([r, p, y])
    sr, sp, sy = np.sin([r, p, y])
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


@dataclass
class Shape:
    """Store one conservative shape in a named URDF link, with fixed origin."""

    kind: str
    offset: np.ndarray
    basis: np.ndarray
    lower: np.ndarray
    upper: np.ndarray

    def contains(self, points, padding):
        """Include the padded surface; mesh boxes may also exclude empty space."""
        local = (points-self.offset) @ self.basis
        if self.kind == 'sphere':
            return np.linalg.norm(local, axis=1) <= self.upper[0]+padding
        if self.kind == 'cylinder':
            return ((np.linalg.norm(local[:, :2], axis=1) <= self.upper[0]+padding)
                    & (np.abs(local[:, 2]) <= self.upper[2]+padding))
        return ((local >= self.lower-padding) & (local <= self.upper+padding)).all(axis=1)


class RobotBody:
    """Load all rendered and collidable shapes; unsupported geometry fails closed."""

    def __init__(self, text, mesh_root, catalog=None, padding=0.02):
        """Verify native mesh bounds against actual installed mesh SHA-256 values."""
        if not np.isfinite(padding) or not 0.01 <= padding <= 0.05:
            raise ValueError('invalid_self_padding')
        if len(text) > 3000000:
            raise ValueError('oversized_robot_description')
        root = ET.fromstring(text)
        if root.tag != 'robot':
            raise ValueError('not_a_robot_description')
        self.digest = hashlib.sha256(text.encode()).hexdigest()
        self.padding = padding
        self.mesh_root = Path(mesh_root).resolve()
        self.catalog = (catalog if catalog is not None else json.loads(
            files('guidedog_perception').joinpath('mesh_bounds.json').read_text())['meshes'])
        self.meshes = {}
        self.links = {}
        for link in root.findall('link'):
            name = link.get('name')
            if not name or name in self.links:
                raise ValueError('invalid_or_duplicate_link')
            shapes = []
            for element in list(link.findall('visual'))+list(link.findall('collision')):
                origin = element.find('origin')
                xyz = vector(origin.get('xyz') if origin is not None else None, 3, '0 0 0')
                rpy = vector(origin.get('rpy') if origin is not None else None, 3, '0 0 0')
                geometry = element.find('geometry')
                if geometry is None or len(geometry) != 1:
                    raise ValueError('missing_or_ambiguous_geometry')
                item = geometry[0]
                if item.tag == 'mesh':
                    lower, upper = self.mesh_bounds(item)
                elif item.tag == 'box':
                    size = vector(item.get('size'), 3)
                    lower, upper = -size/2, size/2
                elif item.tag == 'sphere':
                    upper = np.full(3, float(item.get('radius')))
                    lower = -upper
                elif item.tag == 'cylinder':
                    radius, half = float(item.get('radius')), float(item.get('length'))/2
                    upper = np.array([radius, radius, half])
                    lower = -upper
                else:
                    raise ValueError('unsupported_robot_geometry:'+item.tag)
                if (not np.isfinite([lower, upper]).all() or not (upper > lower).all()
                        or np.max(upper-lower) > 5):
                    raise ValueError('invalid_robot_geometry_bounds')
                shapes.append(Shape(item.tag, xyz, rotation(rpy), lower, upper))
            self.links[name] = shapes
        self.links = {name: shapes for name, shapes in self.links.items() if shapes}
        if not self.links or sum(map(len, self.links.values())) > 100:
            raise ValueError('missing_or_oversized_robot_geometry')

    def mesh_bounds(self, item):
        """Accept only catalogued files beneath the declared mesh root, never a guess."""
        uri = urlparse(item.get('filename', ''))
        if uri.scheme == 'package' and uri.netloc == 'unitree_go2_description':
            path = self.mesh_root / Path(unquote(uri.path)).name
            if unquote(uri.path) != '/meshes/'+path.name:
                raise ValueError('mesh_outside_catalog_root')
        elif uri.scheme == 'file' and not uri.netloc:
            path = Path(unquote(uri.path))
        else:
            raise ValueError('unsupported_mesh_uri')
        path = path.resolve()
        # ament symlink-install may link each file while leaving meshes/ itself
        # a real directory. Bind to that exact installed asset, then verify bytes.
        if path != (self.mesh_root/path.name).resolve() or uri.query or uri.fragment:
            raise ValueError('mesh_outside_catalog_root')
        scale = vector(item.get('scale'), 3, '1 1 1')
        if not ((scale > 0) & (scale <= 100)).all():
            raise ValueError('invalid_mesh_scale')
        if path.name not in self.meshes:
            known = self.catalog.get(path.name)
            if known is None or path.stat().st_size > 20000000:
                raise ValueError('uncatalogued_or_oversized_mesh')
            if hashlib.sha256(path.read_bytes()).hexdigest() != known['sha256']:
                raise ValueError('mesh_hash_mismatch:'+path.name)
            self.meshes[path.name] = known
        known = self.meshes[path.name]
        return np.array(known['min'])*scale, np.array(known['max'])*scale

    def mask(self, points, transforms):
        """Require each link at the cloud's exact stamp; never reuse a partial pose."""
        points = np.asarray(points, dtype=float)
        if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
            raise ValueError('invalid_self_filter_points')
        if set(transforms) != set(self.links):
            raise ValueError('incomplete_self_transforms')
        removed = np.zeros(len(points), dtype=bool)
        for name, shapes in self.links.items():
            local = transform_xyz(points, *transforms[name])
            for shape in shapes:
                removed |= shape.contains(local, self.padding)
        return removed

    def summary(self):
        """Describe the model assumptions without implying exact mesh intersection."""
        return {'model_sha256': self.digest, 'padding_m': self.padding,
                'links': len(self.links), 'shapes': sum(map(len, self.links.values())),
                'mesh_sha256': {k: v['sha256'] for k, v in self.meshes.items()},
                'mesh_method': 'native Gazebo mesh bounds; conservative boxes, not triangles',
                'scope': 'endpoint containment; no ray-shadow or continuous-surface proof'}
