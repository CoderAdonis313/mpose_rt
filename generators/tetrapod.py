"""Eight asymmetric 3D fiducial families for MegaPose evaluation.

Blender 3.6+/4.x: blender -b --python marker_generator.py -- --design all --out ./outputs
Single: blender -b --python marker_generator.py -- --design radial_star --out ./outputs
Coordinates: meters, Z upward, mounting plane Z=0, marker mount origin=(0,0,0).
All designs are designed for <= 0.10m XYZ bounding envelope, subject to export check.
"""
import bpy
import math
import os
import sys
import argparse
from mathutils import Vector

DESIGNS = ('radial_star','tetrapod','orthogonal_branches','stepped_tower',
           'asymmetric_fins','lopsided_cage','multiscale_polyhedron','bent_spokes')
PALETTE = [(0.90,0.24,0.12,1), (0.13,0.58,0.87,1), (0.98,0.78,0.13,1),
           (0.15,0.72,0.43,1), (0.71,0.32,0.87,1), (0.95,0.52,0.18,1),
           (0.28,0.32,0.77,1), (0.77,0.19,0.43,1)]


def clean():
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    for m in list(bpy.data.materials):
        if m.name.startswith('Marker_'):
            bpy.data.materials.remove(m)


def mat(i):
    name = f'Marker_{i:02d}'
    m = bpy.data.materials.get(name)
    if m is None:
        m = bpy.data.materials.new(name)
        m.diffuse_color = PALETTE[i % len(PALETTE)]
        m.use_nodes = True
        node = m.node_tree.nodes.get('Principled BSDF')
        if node:
            node.inputs['Base Color'].default_value = m.diffuse_color
            node.inputs['Roughness'].default_value = 0.7
            node.inputs['Metallic'].default_value = 0.0
    return m


def finish(obj, color=0):
    obj.data.materials.clear()
    obj.data.materials.append(mat(color))
    return obj


def box(name, location, scale, color=0):
    bpy.ops.mesh.primitive_cube_add(size=1, location=location)
    o = bpy.context.object; o.name=name
    o.dimensions = scale
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    return finish(o,color)


def uv_sphere(name, center, radius, color=0):
    bpy.ops.mesh.primitive_uv_sphere_add(segments=20, ring_count=12, radius=radius, location=center)
    o=bpy.context.object; o.name=name
    bpy.ops.object.shade_smooth()
    return finish(o,color)


def ico(name, center, radius, subdivisions=1, color=0):
    bpy.ops.mesh.primitive_ico_sphere_add(subdivisions=subdivisions, radius=radius, location=center)
    o=bpy.context.object; o.name=name
    return finish(o,color)


def rod(name, start, end, radius=0.006, color=0, vertices=12):
    a,b=Vector(start),Vector(end)
    direction = b-a
    assert direction.length > 1e-6, name
    middle=(a+b)/2
    bpy.ops.mesh.primitive_cylinder_add(vertices=vertices, radius=radius, depth=direction.length,
                                        end_fill_type='NGON', location=middle)
    o=bpy.context.object; o.name=name
    o.rotation_euler = direction.to_track_quat('Z','Y').to_euler()
    return finish(o,color)


def cone(name, base, tip, radius1=0.01, radius2=0.001, color=0, vertices=16):
    a,b=Vector(base),Vector(tip)
    direction=b-a
    bpy.ops.mesh.primitive_cone_add(vertices=vertices, radius1=radius1, radius2=radius2,
                                    depth=direction.length, location=(a+b)/2)
    o=bpy.context.object; o.name=name
    o.rotation_euler=direction.to_track_quat('Z','Y').to_euler()
    return finish(o,color)


def panel(name, verts, thickness=0.004, color=0):
    """Closed triangular or quadrilateral fin with thickness on both sides."""
    points=[Vector(v) for v in verts]
    n=len(points)
    normal=(points[1]-points[0]).cross(points[2]-points[0]).normalized()
    half=normal*thickness/2
    coordinates=[p+half for p in points]+[p-half for p in points]
    faces=[tuple(range(n)), tuple(range(2*n-1,n-1,-1))]
    for i in range(n):
        j=(i+1)%n
        faces.append((i,j,j+n,i+n))
    mesh=bpy.data.meshes.new(name+'_mesh')
    mesh.from_pydata(coordinates, [], faces)
    mesh.update()
    ob=bpy.data.objects.new(name,mesh)
    bpy.context.collection.objects.link(ob)
    return finish(ob,color)


def base_plate():
    # Thin robot mounting plate; center at (0,0), top at z=0.008m.
    box('mounting_plate', (0,0,0.004), (0.035,0.035,0.008), 6)


def radial_star():
    base_plate(); c=Vector((0,0,0.049))
    uv_sphere('star_hub',c,0.012,0)
    # Intentionally non-opposite, uneven angular/length distribution.
    dirs=[(1,.08,.36),(-.55,.92,.19),(.25,-1,.44),(-.8,-.55,-.20),
          (.32,.37,1),(-.18,.20,-1.0)]
    lens=[.035,.044,.040,.037,.031,.030]
    for i,(d,L) in enumerate(zip(dirs,lens)):
        u=Vector(d).normalized(); a=c+u*.008; b=c+u*L
        rod(f'star_arm_{i}',a,b,[.007,.0055,.0065,.006,.005,.007][i],i+1)
        if i in (0,3):
            uv_sphere(f'star_tip_{i}',b,.008,i+1)


def tetrapod():
    base_plate(); c=Vector((0,0,.051))
    ico('tetra_hub',c,.013,1,0)
    dirs=[(1,.15,.17),(-.32,1,.4),(-.55,-.31,.9),(.33,-.76,-.70)]
    lens=[.039,.034,.041,.033]
    for i,(d,L) in enumerate(zip(dirs,lens)):
        u=Vector(d).normalized(); a=c+u*.008; b=c+u*L
        rod(f'tetrapod_arm_{i}',a,b,.008-0.0007*i,i+1)
        ico(f'tetrapod_tip_{i}',b,.009-0.001*i,1,i+1)


def orthogonal_branches():
    base_plate(); c=Vector((0,0,.050))
    box('main_hub',c,(.024,.018,.024),0)
    specs=[((1,0,0),.033,.013),((-1,0,0),.021,.018),((0,1,0),.036,.010),
           ((0,-1,0),.017,.013),((0,0,1),.032,.014),((0,0,-1),.023,.016)]
    for i,(axis,L,w) in enumerate(specs):
        a=Vector(axis); p=c+a*(L/2+.006)
        dims=tuple(L if abs(v)>0 else w for v in axis)
        box(f'branch_{i}',p,dims,i+1)
    # Shifted notch-like protrusion to further disambiguate orientation.
    box('branch_offset_flag',(.037,.007,.052),(.008,.010,.012),3)


def stepped_tower():
    base_plate()
    blocks=[((.000,.000,.020),(.032,.029,.025)),
            ((.008,-.003,.044),(.028,.021,.025)),
            ((-.003,.008,.067),(.025,.026,.022)),
            ((.012,.005,.085),(.018,.015,.014))]
    for i,(p,scale) in enumerate(blocks):
        box(f'step_{i}',p,scale,i)
    box('side_boss',(-.020,.008,.052),(.012,.010,.012),5)


def asymmetric_fins():
    base_plate()
    box('fin_spine',(0,0,.047),(.015,.016,.073),0)
    # Triangular fin vertices, thickness; broad continuous color patches in RGB.
    fins=[[(0,0,.029),(.043,.004,.034),(.005,.008,.087)],
          [(-.004,0,.022),(-.027,-.030,.048),(-.007,-.002,.073)],
          [(.003,0,.039),(.008,.039,.052),(-.012,.010,.083)],
          [(0,-.005,.021),(.023,-.024,.026),(.006,-.009,.066)]]
    for i,vertices in enumerate(fins):
        panel(f'fin_{i}',vertices,thickness=.005,color=i+1)


def lopsided_cage():
    base_plate()
    corners=[(-.030,-.024,.019),(.034,-.024,.019),(.034,.031,.019),(-.030,.031,.019),
             (-.025,-.029,.081),(.026,-.020,.091),(.032,.027,.072),(-.028,.023,.087)]
    for i,j in [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),
                (0,4),(1,5),(2,6),(3,7),(4,2)]:
        rod(f'cage_edge_{i}_{j}',corners[i],corners[j],.0048,(i+j)%8)
    uv_sphere('cage_distinct_top_corner',corners[5],.009,1)
    ico('cage_distinct_side_corner',corners[3],.008,1,4)


def multiscale_polyhedron():
    base_plate()
    c=Vector((0,0,.050))
    ico('main_polyhedron',c,.025,1,0)
    # One broad asymmetric branch and three smaller facet-related landmarks.
    cone('large_conical_boss',c+Vector((.012,.006,.005)),c+Vector((.042,.011,.020)),.012,.005,1)
    rod('small_boss_A',c+Vector((-.014,.006,.010)),c+Vector((-.031,.012,.024)),.006,2)
    box('small_boss_B',c+Vector((.005,-.023,-.007)),(.015,.016,.018),3)
    ico('small_boss_C',c+Vector((-.013,-.005,-.029)),.009,1,4)
    box('top_flat_feature',c+Vector((.000,.009,.027)),(.012,.015,.007),5)


def bent_spokes():
    base_plate(); c=Vector((0,0,.052))
    uv_sphere('elbow_hub',c,.012,0)
    joints=[((.019,.004,.013),(.034,.019,.018)),
            ((-.018,.020,.008),(-.026,.034,-.002)),
            ((-.020,-.014,-.007),(-.037,-.018,.009)),
            ((.012,-.021,-.013),(.025,-.033,-.020)),
            ((.006,.008,.024),(-.002,.019,.035))]
    for i,(j,t) in enumerate(joints):
        mid=c+Vector(j); tip=c+Vector(t)
        rod(f'bent_root_{i}',c,mid,.0065,i+1)
        rod(f'bent_end_{i}',mid,tip,.0055,i+1)
        uv_sphere(f'bent_elbow_{i}',mid,.006,i+1)


BUILDERS={n:globals()[n] for n in DESIGNS}


def bounds(objects):
    # Evaluated world-space bounds from object bound_boxes.
    corners=[o.matrix_world@Vector(c) for o in objects if o.type=='MESH' for c in o.bound_box]
    lo=[min(v[k] for v in corners) for k in range(3)]
    hi=[max(v[k] for v in corners) for k in range(3)]
    return lo,hi


def export_glb(path):
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.export_scene.gltf(filepath=path,export_format='GLB',use_selection=False,
                              export_apply=True, export_yup=True)


def export_ply(path):
    """Write ASCII PLY in Blender XYZ coordinates (meters), triangular faces.
    Uses objects independently and bakes each object's world transform into vertices.
    """
    vertices=[]; faces=[]
    depsgraph=bpy.context.evaluated_depsgraph_get()
    for obj in bpy.context.scene.objects:
        if obj.type != 'MESH': continue
        evaluated=obj.evaluated_get(depsgraph)
        mesh=evaluated.to_mesh()
        try:
            mesh.calc_loop_triangles()
            start=len(vertices)
            vertices.extend(tuple(evaluated.matrix_world@v.co) for v in mesh.vertices)
            faces.extend(tuple(start+k for k in t.vertices) for t in mesh.loop_triangles)
        finally:
            evaluated.to_mesh_clear()
    with open(path,'w',encoding='utf8') as f:
        f.write('ply\nformat ascii 1.0\ncomment units meters; Z-up; origin at mounting plane\n')
        f.write(f'element vertex {len(vertices)}\nproperty float x\nproperty float y\nproperty float z\n')
        f.write(f'element face {len(faces)}\nproperty list uchar int vertex_indices\nend_header\n')
        for v in vertices: f.write(f'{v[0]:.8f} {v[1]:.8f} {v[2]:.8f}\n')
        for a,b,c in faces: f.write(f'3 {a} {b} {c}\n')


def main():
    arg=sys.argv[sys.argv.index('--')+1:] if '--' in sys.argv else []
    p=argparse.ArgumentParser()
    p.add_argument('--design',choices=('all',)+DESIGNS,default='tetrapod')
    p.add_argument('--out',default=os.path.join(os.path.dirname(os.path.abspath(__file__)),'outputs'))
    p.add_argument('--save-blend',action='store_true')
    a=p.parse_args(arg)
    out=os.path.abspath(a.out); os.makedirs(out,exist_ok=True)
    choices=DESIGNS if a.design=='all' else [a.design]
    for name in choices:
        clean()
        BUILDERS[name]()
        bpy.context.view_layer.update()
        lo,hi=bounds(list(bpy.context.scene.objects))
        print(f'[{name}] Bounds min={lo}, max={hi} (meters)')
        if any(hi[k]-lo[k]>.1001 for k in range(3)):
            raise RuntimeError(f'{name}: exceeded 100mm dimension')
        if lo[2] < -0.0001 or hi[2]>.1001:
            raise RuntimeError(f'{name}: violated Z=0..100mm envelope')
        if a.save_blend:
            bpy.ops.wm.save_as_mainfile(filepath=os.path.join(out,name+'.blend'))
        export_glb(os.path.join(out,name+'.glb'))
        export_ply(os.path.join(out,name+'.ply'))
        print(f'[{name}] Saved GLB + PLY in {out}')

if __name__=='__main__':
    main()
