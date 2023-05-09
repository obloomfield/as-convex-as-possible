
mesh_obj = bpy.data.objects.new(name="my_plane_object", object_data=mesh_data)
bpy.context.scene.collection.objects.link(mesh_obj)
