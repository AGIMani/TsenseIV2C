"""Offscreen, full-triangle renders for the shell URDF repair review."""
from pathlib import Path
import argparse
import xml.etree.ElementTree as ET

import moderngl
import numpy as np
from PIL import Image, ImageDraw
from scipy.spatial.transform import Rotation
import trimesh


def transforms(root, pose=None):
    pose = pose or {}
    joints = root.findall('joint')
    children = {j.find('child').get('link') for j in joints}
    base = next(l.get('name') for l in root.findall('link') if l.get('name') not in children)
    result = {base: np.eye(4)}
    pending = joints[:]
    while pending:
        remaining = len(pending)
        for j in pending[:]:
            parent, child = j.find('parent').get('link'), j.find('child').get('link')
            if parent not in result:
                continue
            t = origin(j.find('origin'))
            if j.get('type') != 'fixed':
                a = np.fromstring(j.find('axis').get('xyz'), sep=' ')
                t[:3, :3] = t[:3, :3] @ Rotation.from_rotvec(a * pose.get(j.get('name'), 0)).as_matrix()
            result[child] = result[parent] @ t
            pending.remove(j)
        if len(pending) == remaining:
            raise ValueError('Disconnected or cyclic URDF')
    return result


def origin(node):
    result = np.eye(4)
    if node is not None:
        result[:3, :3] = Rotation.from_euler('xyz', np.fromstring(node.get('rpy', '0 0 0'), sep=' ')).as_matrix()
        result[:3, 3] = np.fromstring(node.get('xyz', '0 0 0'), sep=' ')
    return result


def parts(path, pose=None, palm_override=None, root=None):
    if path.suffix.lower() == '.stl':
        return [(trimesh.load_mesh(path, process=False), (0.72, 0.77, 0.82))]
    root = ET.parse(path).getroot() if root is None else root
    tf = transforms(root, pose)
    result = []
    for link in root.findall('link'):
        for visual in link.findall('visual'):
            mesh = visual.find('geometry/mesh')
            if mesh is None:
                continue
            name = mesh.get('filename')
            filename = (path.parent / name).resolve()
            if name.startswith('package://'):
                filename = path.parent.parent / 'meshes' / Path(name).name
            if palm_override and link.get('name') in ('base_link', 'Left_hand_palm_link'):
                filename = palm_override
            m = trimesh.load_mesh(filename, process=False)
            m.apply_transform(tf[link.get('name')] @ origin(visual.find('origin')))
            color = visual.find('material/color')
            rgb = tuple(map(float, color.get('rgba').split()[:3])) if color is not None else (0.7, 0.75, 0.8)
            result.append((m, rgb))
    return result


def render(objects, camera, size=(650, 800)):
    ctx = moderngl.create_standalone_context(require=330)
    program = ctx.program(vertex_shader='''#version 330
        in vec3 position; in vec3 normal; uniform mat4 projection;
        out vec3 n; void main(){gl_Position=projection*vec4(position,1); n=normal;}''',
        fragment_shader='''#version 330
        in vec3 n; uniform vec3 light; uniform vec3 color; out vec4 f;
        void main(){vec3 nn=normalize(n); float d=max(dot(nn,light),0.);
        f=vec4(color*(.25+.75*d),1);}''')
    target = ctx.simple_framebuffer(size, components=4, samples=min(4, ctx.max_samples))
    resolved = ctx.simple_framebuffer(size, components=4)
    target.use(); ctx.enable(moderngl.DEPTH_TEST | moderngl.CULL_FACE)
    target.clear(.94, .95, .97, 1)
    bounds = np.array([m.bounds for m, _ in objects]); lo, hi = bounds[:, 0].min(0), bounds[:, 1].max(0)
    center = (lo + hi) / 2
    cam = np.asarray(camera, float); cam /= np.linalg.norm(cam)
    right = np.cross([0, 0, 1], cam); right /= np.linalg.norm(right)
    up = np.cross(cam, right)
    span = max(hi[2] - lo[2], (hi[0]-lo[0]) * size[1]/size[0]) * 1.17
    projection = np.eye(4)
    projection[0, :3] = right * 2 / span * size[1]/size[0]
    projection[1, :3] = up * 2 / span
    projection[2, :3] = -cam / span
    projection[:3, 3] = -projection[:3, :3] @ center
    program['projection'].write(projection.astype('f4').T.tobytes())
    light = cam + up * .5 - right * .3; light /= np.linalg.norm(light)
    program['light'].value = tuple(light)
    for m, color in objects:
        v = np.asarray(m.triangles, dtype='f4')
        nn = np.cross(v[:, 1]-v[:, 0], v[:, 2]-v[:, 0]); nn /= np.maximum(np.linalg.norm(nn, axis=1)[:, None], 1e-20)
        data = np.c_[v.reshape(-1, 3), np.repeat(nn, 3, axis=0)].astype('f4')
        buffer = ctx.buffer(data.tobytes())
        vao = ctx.vertex_array(program, [(buffer, '3f 3f', 'position', 'normal')])
        program['color'].value = color
        vao.render(moderngl.TRIANGLES)
        vao.release(); buffer.release()
    ctx.copy_framebuffer(resolved, target)
    result = Image.frombytes('RGBA', size, resolved.read(components=4, alignment=1)).transpose(Image.Transpose.FLIP_TOP_BOTTOM).convert('RGB')
    target.release(); resolved.release(); program.release(); ctx.release()
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('source', type=Path); p.add_argument('output', type=Path)
    p.add_argument('--palm', type=Path); p.add_argument('--curl', action='store_true')
    p.add_argument('--reference-frame', action='store_true', help='View a model rebased to bare A3.0 axes')
    args = p.parse_args()
    pose = {}
    if args.curl:
        for finger in ('index', 'mid', 'ring', 'pinky'):
            for joint, value in [('MCP', .6), ('PIP', .7), ('DIP', .5)]:
                pose[f'left_hand_{finger}_{joint}_joint'] = value
        pose.update(left_hand_thumb_MCP_joint=.4, left_hand_thumb_PIP_joint=.4)
    obj = parts(args.source, pose, args.palm)
    out = Image.new('RGB', (1950, 840), 'white')
    for i, (cam, label) in enumerate([((.2,-1,.15), 'Palm'), ((-.55,-1,.35), 'Oblique'), ((.2,1,.15), 'Back')]):
        if args.reference_frame:
            cam = (-cam[1], cam[0], cam[2])
        out.paste(render(obj, cam), (650*i, 40)); ImageDraw.Draw(out).text((650*i+20, 12), label, fill='black')
    args.output.parent.mkdir(parents=True, exist_ok=True); out.save(args.output)
    print(args.output)


if __name__ == '__main__':
    main()
