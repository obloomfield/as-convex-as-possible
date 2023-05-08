import bpy

mesh_data = bpy.data.meshes.new(name="my_plane_mesh")

verts = [
    (-1.2, 1.6, 1.5),
    (1.4, -1.3, 1.5),
    (-1.2, 1.6, -1.005929),
    (1.4, -1.3, -1.005929),
]

for v in verts:
    mesh_data.vertices.add(v)

faces = [
    (0, 1, 3, 2),
]

for f in faces:
    mesh_data.faces.new(f)


mesh_obj = bpy.data.objects.new(name="my_plane_object", object_data=mesh_data)
bpy.context.scene.collection.objects.link(mesh_obj)
